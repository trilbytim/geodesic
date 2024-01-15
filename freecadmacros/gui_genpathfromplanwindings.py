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

def repeatwindingpath(rpts, repeats, thintol, angadvanceperwind, initialangadvance):
    Dptfront, Dptback = rpts[0], rpts[-1]
    Dfvec0 = P2(Dptfront.x, Dptfront.z)
    Dfvec1 = P2(Dptback.x, Dptback.z)
    Dangadvance = P2(P2.Dot(Dfvec0, Dfvec1), P2.Dot(Dfvec0, P2.APerp(Dfvec1))).Arg()
    Dangadvanceperwind = P2(P2.Dot(Dfvec0, Dfvec1), P2.Dot(Dfvec0, P2.APerp(Dfvec1))).Arg()
    Dangdiff = min(abs(((Dangadvanceperwind+Da)%360) - ((angadvanceperwind+Da)%360))  for Da in [360, 360+180])
    TOL_ZERO(Dangdiff, "Dangdiff")
    rpts = thinptstotolerance(rpts, tol=thintol*0.5)
    ptsout = [ ]
    for i in range(repeats):
        rotcos = math.cos(math.radians(i*angadvanceperwind + initialangadvance))
        rotsin = math.sin(math.radians(i*angadvanceperwind + initialangadvance))
        for pt in rpts:
            ptsout.append(P3(pt.x*rotcos + pt.z*rotsin, pt.y, pt.z*rotcos - pt.x*rotsin))
    ptsout = thinptstotolerance(ptsout, tol=thintol)
    return ptsout



class GenPathFromPlanWindingsTaskPanel(QtGui.QWidget):
    def __init__(self):
        x = os.path.join(os.path.split(__file__)[0], "genpathfromplanwindingstask.ui")
        self.form = FreeCADGui.PySideUic.loadUi(x)
        self.form.setMinimumSize(0, 420)
        self.update()

    def update(self):
        self.doc = App.ActiveDocument
        self.sel = App.Gui.Selection.getSelection()
        planwindings = sgetlabelofselectedgroupwithproperties(self.sel, ["planwindingstype"])
        outputwindings = sgetlabelofselectedgroupwithproperties(self.sel, ["outputwindingstype"])
        if planwindings:
            self.form.qplanwindings.setText(planwindings)
        if outputwindings:
            self.form.qoutputwindings.setText(outputwindings)
        planwindingsgroup = sfindobjectbylabel(self.doc, self.form.qplanwindings.text())
        if planwindingsgroup:
            self.form.qsketchplane.setText(planwindingsgroup.sketchplane)
            self.form.qmeshobject.setText(planwindingsgroup.meshobject)
            self.form.qalongwire.setValue(planwindingsgroup.alongwire)
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
        
    def apply(self):
        print("apply!!")
        alongwire = float(self.form.qalongwire.text())
        thinningtol = float(self.form.qthinningtol.text())
        planwindingsgroup = sfindobjectbylabel(self.doc, self.form.qplanwindings.text())
        optiontestminimal = self.form.qoptiontestminimal.isChecked()
        if planwindingsgroup == None:
            print("No planwindings selected")
            return
        outputwindingsgroup = freecadutils.getemptyfolder(self.doc, self.form.qoutputwindings.text())
        setpropertyval(planwindingsgroup, "App::PropertyString", "planwindings", planwindingsgroup.Label)
        setpropertyval(planwindingsgroup, "App::PropertyBool", "outputwindingsgrouptype", True)
        setpropertyval(planwindingsgroup, "App::PropertyFloat", "thinningtol", thinningtol)

        initialangadvance = 0.0
        for planwinding in planwindingsgroup.OutList:
            adjustedalongwirelanded, adjustedwindings = adjustlandingrepeatstobecoprime(planwinding.alongwire, planwinding.alongwirelanded, planwinding.plannedwinds)
            Dadustedalongwireadvance = adjustedalongwirelanded - alongwire
            if Dadustedalongwireadvance <= 0.0:
                Dadustedalongwireadvance += 1.0
            print("turns", round(Dadustedalongwireadvance*adjustedwindings), "for windings", adjustedwindings)
            gbs, fLRdirection, dseg, Dalongwirelanded = directedgeodesic(combofoldbackmode, self.drivecurve, self.utbm, planwinding.alongwire, adjustedalongwirelanded, float(planwinding.splayangle), Maxsideslipturningfactor, mandrelradius, 0.0, maxlength, None)
       
            angadvanceperwind = (adjustedalongwirelanded - planwinding.alongwire + 1.0)*360
            if optiontestminimal:
                adjustedwindings = 1
            rpts = repeatwindingpath([P3(*gb.pt)  for gb in gbs], adjustedwindings, thinningtol, angadvanceperwind, initialangadvance)
            
            if optiontestminimal:
                initialangadvance += (adjustedwindings*angadvanceperwind) % 360
            else:
                TOL_ZERO(((((adjustedwindings*angadvanceperwind)%360) + 180) % 360) - 180, "advanceperwind")

            name = 'w%dx%d' % (90-int(planwinding.splayangle), adjustedwindings)
            ply = Part.show(Part.makePolygon([Vector(pt)  for pt in rpts]), name)
            outputwindingsgroup.addObject(ply)
            setpropertyval(ply, "App::PropertyAngle", "splayangle", planwinding.splayangle)
            setpropertyval(ply, "App::PropertyFloat", "alongwire", alongwire)
            setpropertyval(ply, "App::PropertyFloat", "alongwirelanded", adjustedalongwirelanded)
            setpropertyval(ply, "App::PropertyInteger", "windings", adjustedwindings)
            setpropertyval(ply, "App::PropertyFloat", "towwidth", planwinding.towwidth)
            setpropertyval(ply, "App::PropertyFloat", "towthickness", planwinding.towthickness)


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

Gui.Control.showDialog(GenPathFromPlanWindingsTaskPanel())
