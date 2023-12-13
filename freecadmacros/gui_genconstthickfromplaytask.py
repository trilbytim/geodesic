# -*- coding: utf-8 -*-

# stock viewer task, a standard interactive panel on the left in FreeCAD.  
# To edit the UI file, nix-shell -p qtcreator, start up qtcreator
# select default current project and start editing stockviewertask.ui


import Draft, Part, Mesh, MeshPart, Fem
from FreeCAD import Vector, Rotation 
from PySide import QtGui, QtCore


import os, sys, math, time, numpy
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])


from barmesh.basicgeo import I1, Partition1, P3, P2, Along, lI1
from barmesh.tribarmes.triangleboxing import TriangleBoxing

import imp
import utils.directedgeodesic
imp.reload(utils.directedgeodesic)
import utils.pathutils
imp.reload(utils.pathutils)

from utils.pathutils import BallPathCloseRegions, MandrelPaths, MakeFEAcoloredmesh
import utils.freecadutils as freecadutils
freecadutils.init(App)
from utils.curvesutils import cumlengthlist, seglampos


from utils.directedgeodesic import directedgeodesic, makebicolouredwire, makedrivecurve, drivegeodesicRI, DriveCurve
from utils.trianglemeshutils import UsefulBoxedTriangleMesh
from utils.geodesicutils import drivegeodesic, InvAlong, GBarT, GBarC, drivecurveintersectionfinder, trilinecrossing, TOL_ZERO
from utils.curvesutils import thinptstotolerance

import FreeCADGui as Gui
import PySide.QtGui as QtGui
import PySide.QtCore as QtCore
import numpy

def sgetlabelofselectedmesh(sel):
    for s in sel:
        if hasattr(s, "Mesh") and isinstance(s.Mesh, Mesh.Mesh):
            return s.Label
        if s.TypeId == "Mesh::Curvature":
            if "ValueAtIndex" in s.PropertiesList:
                return s.Label
            else:
                print("***  Wrong version of FreeCAD, \n\nMeshCurvature object missing ValueAtIndex hack.")
                return "**WrongFCversion"
    return ""

def sgetlabelofselectedsketch(sel):
    for s in sel:
        if hasattr(s, "Module") and s.Module == 'Sketcher':
            return s.Label
    return ""

def sgetlabelofselectedgroupwithproperties(sel, properties):
    for s in sel:
        if isinstance(s, App.DocumentObjectGroup) and set(properties).issubset(s.PropertiesList):
            return s.Label
    return ""

def sfindobjectbylabel(doc, lab):
    objs = [ obj  for obj in doc.findObjects(Label=lab)  if obj.Label == lab ]
    return objs[0] if objs else None

def setpropertyval(obj, atype, name, value):
    if name not in obj.PropertiesList:
        obj.addProperty(atype, name, "filwind")
    setattr(obj, name, value)


mandrelradius = 110  # fc6 file
LRdirection = 1
appaturepapproachpoint = P3(0,-150,0)
maxlength = 2500

Maxsideslipturningfactor = 0.26
combofoldbackmode = 0


def adjustlandingrepeatstobecoprime(alongwire, alongwirelanded, totalrepeats):
    alongwireadvance = alongwirelanded - alongwire
    if alongwireadvance <= 0.0:
        alongwireadvance += 1.0
    assert alongwireadvance > 0
    totalturns = round(alongwireadvance*totalrepeats)
    
    for ttdiff in range(10):
        if math.gcd(totalturns, totalrepeats + ttdiff) == 1:
            newtotalrepeats = totalrepeats + ttdiff
            break
        elif math.gcd(totalturns, totalrepeats - ttdiff) == 1:
            newtotalrepeats = totalrepeats - ttdiff
            break
    else:
        print("didn't find non coprime total turns ", totalturns, totalrepeats)
    newalongwireadvance =  totalturns / newtotalrepeats
    newalongwirelanded = (alongwire + newalongwireadvance) % 1
    return newalongwirelanded, newtotalrepeats

def repeatwindingpath(rpts, repeats,thintol):
    ptfront, ptback = rpts[0], rpts[-1]
    fvec0 = P2(ptfront.x, ptfront.z)
    fvec1 = P2(ptback.x, ptback.z)
    angadvance = P2(P2.Dot(fvec0, fvec1), P2.Dot(fvec0, P2.APerp(fvec1))).Arg()
    rpts = thinptstotolerance(rpts, tol=thintol*2)
    ptsout = rpts[:]
    for i in range(1, repeats):
        rotcos = math.cos(math.radians(i*angadvance))
        rotsin = math.sin(math.radians(i*angadvance))
        for pt in rpts:
            ptsout.append(P3(pt.x*rotcos + pt.z*rotsin, pt.y, pt.z*rotcos - pt.x*rotsin))
    ptsout = thinptstotolerance(ptsout, tol=thintol)
    return ptsout

def evalthick(splaycircle, mandrelptpaths, towwidth, towthickness):
    towrad = towwidth/2
    POpts = [ P3(v.X,v.Y,v.Z)  for v in splaycircle.Shape.Vertexes ]
    POnorms = [ App.Vector(0,1,0) ]*len(POpts)

    mandpaths = MandrelPaths(mandrelptpaths, towrad)
    thickcount = [ ]
    for mp in POpts:
        thickcount.append(mandpaths.BallCloseCount(mp, towrad))
    meanthick = numpy.mean(thickcount)*towthickness
    print('THICKNESSES AROUND RING:')
    print('MAX:',numpy.max(thickcount)*towthickness,'MIN:',numpy.min(thickcount)*towthickness,'MEAN:', meanthick)
    return meanthick

def wirestopatharrays(wirelist):
    basepts = [ ]
    for bw in wirelist:
        if hasattr(bw, "Shape") and isinstance(bw.Shape, Part.Wire) and "towwidth" in bw.PropertiesList:
            print("--evalthick includes:", bw.Name)
            basepts.append([P3(v.X,v.Y,v.Z)  for v in bw.Shape.Vertexes])
    return basepts

def measuremeanthicknessatcircle(mandpaths, towrad, towthickness, splaycircle):
    samplepoints = [ P3(v.X,v.Y,v.Z)  for v in splaycircle.Shape.Vertexes ]
    thicknesscount = [ mandpaths.BallCloseCount(pt, towrad)  for pt in samplepoints ]
    meancount = sum(thicknesscount)/len(thicknesscount)
    return meancount*towthickness


class GenConstThickFromSplayTaskPanel(QtGui.QWidget):
    def __init__(self):
        x = os.path.join(os.path.split(__file__)[0], "genconstthickfromsplaytask.ui")
        self.form = FreeCADGui.PySideUic.loadUi(x)
        self.form.setMinimumSize(0, 390)
        QtCore.QObject.connect(self.form.buttontest, QtCore.SIGNAL("pressed()"), self.testbutton)
        self.update()

    def update(self):
        self.doc = App.ActiveDocument
        self.sel = App.Gui.Selection.getSelection()
        outputwindings = sgetlabelofselectedgroupwithproperties(self.sel, ["splaycircles"])
        if outputwindings:
            self.form.qsplaycircles.setText(outputwindings)
        outputwindingsgroup = sfindobjectbylabel(self.doc, self.form.qoutputwindings.text())
        splaycircles = sgetlabelofselectedgroupwithproperties(self.sel, ["sketchplane", "meshobject"])
        if outputwindings and not splaycircles:
            splaycircles = outputwindings.splaycircles
        if splaycircles:
            self.form.qsplaycircles.setText(splaycircles)
        splaycirclesgroup = sfindobjectbylabel(self.doc, self.form.qsplaycircles.text())
        if splaycirclesgroup:
            self.form.qsketchplane.setText(splaycirclesgroup.sketchplane)
            self.form.qmeshobject.setText(splaycirclesgroup.meshobject)
            self.form.qalongwire.setValue(splaycirclesgroup.alongwire)
        if outputwindingsgroup and len(outputwindingsgroup.OutList):
            self.form.qsphradlimit.setValue(outputwindingsgroup[-1].sphrad)
        elif splaycirclesgroup:
            self.form.qsphradlimitnext.setValue(-1.0 + min(sc.sphrad  for sc in splaycirclesgroup.OutList[-10:]))

        self.sketchplane = sfindobjectbylabel(self.doc, self.form.qsketchplane.text())
        self.meshobject = sfindobjectbylabel(self.doc, self.form.qmeshobject.text())
        if self.meshobject:
            self.utbm = UsefulBoxedTriangleMesh(self.meshobject.Mesh)
            if self.sketchplane:
                self.drivecurve = makedrivecurve(self.sketchplane, self.utbm, mandrelradius)
            else:
                print("Warning, no sketchplane and drivecurve")
        else:
            print("Warning, no meshobject")


    def testbutton(self):
        print("Test button pressed")

    def apply(self):
        print("apply!!")
        splaycirclefolder = sfindobjectbylabel(self.doc, self.form.qsplaycircles.text())
        alongwire = float(self.form.qalongwire.text())
        sphradlimitnext = float(self.form.qsphradlimitnext.text())
        thinningtol = float(self.form.qthinningtol.text())
        towrad = float(self.form.qtowwidth.text())/2.0
        towthickness = float(self.form.qtowthickness.text())
        desiredthickness = float(self.form.qdesiredthickness.text())
        desiredthicknesslower = float(self.form.qdesiredthicknesslower.text())
        outputwindingsgroup = sfindobjectbylabel(self.doc, self.form.qoutputwindings.text())
        if not outputwindingsgroup:
            outputwindingsgroup = freecadutils.getemptyfolder(self.doc, self.form.qoutputwindings.text())
        setpropertyval(outputwindingsgroup, "App::PropertyString", "splaycirclefolder", splaycirclefolder.Label)

        mandpaths = MandrelPaths(wirestopatharrays(outputwindingsgroup.OutList), towrad)

        im = len(splaycirclefolder.OutList)   # small circle is at end
        while im > 0:
            im -= 1

            splaycircle = splaycirclefolder.OutList[im]

            # skip till we find a circle that is wider than the previous
            if splaycircle.sphrad < sphradlimitnext + 0.001:
                continue
                
            meanbasethickness = measuremeanthicknessatcircle(mandpaths, towrad, towthickness, splaycircle)
            print("meanbasethickness", meanbasethickness, "at", splaycircle.sphrad)
            
            # keep going only if the thickness on this circle is within tolerance
            if meanbasethickness <= desiredthicknesslower:
                break

        else:
            print("No next bigger winding to be found")
            return

        # we have now selected our circle level
        
        # estimate thickness for a winding of 100 paths at this hoop layer (tangential to this circle)
        estimatorwindingsnumber = 100
        newalongwirelandedE, newtotalwindingsE = adjustlandingrepeatstobecoprime(splaycircle.alongwire, splaycircle.alongwirelanded, estimatorwindingsnumber)
        gbsE, fLRdirection, dsegE, alongwirelandedE = directedgeodesic(combofoldbackmode, self.drivecurve, self.utbm, alongwire, newalongwirelandedE, float(splaycircle.dsangle), Maxsideslipturningfactor, mandrelradius, 0.0, maxlength, None)
        rptsE = repeatwindingpath([P3(*gb.pt)  for gb in gbsE], newtotalwindingsE, thinningtol)
        mandpathsE = MandrelPaths([rptsE], towrad)
        meanthicknessE = measuremeanthicknessatcircle(mandpathsE, towrad, towthickness, splaycircle)
        additionalthicknessperwinding = meanthicknessE/newtotalwindingsE
        print("meanthicknessE", meanthicknessE)

        requiredadditionalthickness = desiredthickness - meanbasethickness
        requiredwindingsforthickness = int(requiredadditionalthickness/additionalthicknessperwinding) + 1
        adjustedalongwirelanded, adjustedwindings = adjustlandingrepeatstobecoprime(splaycircle.alongwire, splaycircle.alongwirelanded, requiredwindingsforthickness)

        gbs, fLRdirection, dseg, alongwirelanded = directedgeodesic(combofoldbackmode, self.drivecurve, self.utbm, alongwire, adjustedalongwirelanded, float(splaycircle.dsangle), Maxsideslipturningfactor, mandrelradius, 0.0, maxlength, None)
        rpts = repeatwindingpath([P3(*gb.pt)  for gb in gbsE], adjustedwindings, thinningtol)
        name = 'w%dx%d' % (90-int(splaycircle.dsangle), adjustedwindings)
        ply = Part.show(Part.makePolygon([Vector(pt)  for pt in rpts]), name)
        outputwindingsgroup.addObject(ply)
        setpropertyval(ply, "App::PropertyAngle", "sphrad", splaycircle.sphrad)
        setpropertyval(ply, "App::PropertyAngle", "dsangle", splaycircle.dsangle)
        setpropertyval(ply, "App::PropertyFloat", "alongwire", alongwire)
        setpropertyval(ply, "App::PropertyFloat", "alongwirelanded", adjustedalongwirelanded)
        setpropertyval(ply, "App::PropertyInteger", "windings", adjustedwindings)
        setpropertyval(ply, "App::PropertyFloat", "towwidth", towrad*2)

        self.form.qsphradlimitnext.setValue(splaycircle.sphrad)



    def getStandardButtons(self):
        return int(QtGui.QDialogButtonBox.Cancel
                   | QtGui.QDialogButtonBox.Ok
                   | QtGui.QDialogButtonBox.Apply)

    def clicked(self, bt):
        if bt == QtGui.QDialogButtonBox.Apply:
            self.apply()

    def accept(self):
        print("Accept")
        self.apply()
        self.finish()

    def reject(self):
        print("Reject")
        self.finish()

    def finish(self):
        print("Finish")
        Gui.Control.closeDialog()

Gui.Control.showDialog(GenConstThickFromSplayTaskPanel())
