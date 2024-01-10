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

def repeatwindingpath(rpts, repeats, thintol):
    ptfront, ptback = rpts[0], rpts[-1]
    fvec0 = P2(ptfront.x, ptfront.z)
    fvec1 = P2(ptback.x, ptback.z)
    angadvance = P2(P2.Dot(fvec0, fvec1), P2.Dot(fvec0, P2.APerp(fvec1))).Arg()
    print("repeatwindingpath angadvance prop", angadvance/360)
    rpts = thinptstotolerance(rpts, tol=thintol*2)
    ptsout = rpts[:]
    for i in range(1, repeats):
        rotcos = math.cos(math.radians(i*angadvance))
        rotsin = math.sin(math.radians(i*angadvance))
        for pt in rpts:
            ptsout.append(P3(pt.x*rotcos + pt.z*rotsin, pt.y, pt.z*rotcos - pt.x*rotsin))
    ptsout = thinptstotolerance(ptsout, tol=thintol)
    return ptsout


def wirestopatharrays(wirelist):
    basepts = [ ]
    for bw in wirelist:
        if hasattr(bw, "Shape") and isinstance(bw.Shape, Part.Wire):
            basepts.append([P3(v.X,v.Y,v.Z)  for v in bw.Shape.Vertexes])
    return basepts

def measuremeanthicknessatcircle(mandpaths, towrad, towthickness, splaycircle):
    samplepoints = [ P3(v.X,v.Y,v.Z)  for v in splaycircle.Shape.Vertexes ]
    thicknesscount = [ mandpaths.BallCloseCount(pt, towrad)  for pt in samplepoints ]
    meancount = sum(thicknesscount)/len(thicknesscount)
    return meancount*towthickness


def mergeinranges(ranges, rangesI):
    ranges.extend(rangesI)
    ranges.sort(key=lambda X: X.lo)
    i = 0
    while i < len(ranges) - 1:
        if ranges[i+1].lo <= ranges[i].hi:
            ranges[i] = I1(ranges[i].lo, max(ranges[i].hi, ranges[i+1].hi))
            del ranges[i+1]
        else:
            i += 1


def getcirclerangesonpt(circlepaths, pt, towrad):
    bpcr = BallPathCloseRegions(pt, towrad)
    circlepaths.nhitreg += 1
    for ix, iy in circlepaths.tbs.CloseBoxeGenerator(pt.x, pt.x, pt.y, pt.y, towrad):
        tbox = circlepaths.tbs.boxes[ix][iy]
        for i in tbox.pointis:
            bpcr.DistPoint(circlepaths.getpt(i), i)
        for i in tbox.edgeis:
            if circlepaths.hitreg[i] != circlepaths.nhitreg:
                bpcr.DistEdge(circlepaths.getpt(i), circlepaths.getpt(i+1), i)
                circlepaths.hitreg[i] = circlepaths.nhitreg
    bpcr.mergeranges()
    return bpcr.ranges

def calccirclethicknesses(splayhooppts, circlepaths, circlelengths, towrad, towthickness):

    assert len(circlepaths.mandrelptpaths) == len(circlelengths)
    cranges = [ ]
    for pt in splayhooppts: # [ splayhooppts[len(splayhooppts)//2] ]:
            # should do the cylinders connecting these points too, but we will consider the spheres  
            # at the corner points are overlapping enough to have a small enough 
            # cusp to treat it as this at the moment
        crangesI = getcirclerangesonpt(circlepaths, pt, towrad)
        mergeinranges(cranges, crangesI)

    circleoverlapthicknesses = [ 0.0 ] * len(circlelengths)
    for rg in cranges:
        si = int(rg.lo)
        ci = (si//circlepaths.Nm)
        circleoverlapthicknesses[ci] += circlepaths.getgaplength(rg.lo, rg.hi)
    #print(circleoverlapthicknesses)
    circlethicknesses = [ circleoverlapthickness/circlelength*towthickness  for circleoverlapthickness, circlelength in zip(circleoverlapthicknesses, circlelengths) ]
    return circlethicknesses


class GenConstThickOnStockCirclesTaskPanel(QtGui.QWidget):
    def __init__(self):
        x = os.path.join(os.path.split(__file__)[0], "genconstthickonstockcirclestask.ui")
        self.form = FreeCADGui.PySideUic.loadUi(x)
        self.form.setMinimumSize(0, 520)
        QtCore.QObject.connect(self.form.buttontest, QtCore.SIGNAL("pressed()"), self.testbutton)
        self.update()

    def update(self):
        self.doc = App.ActiveDocument
        self.sel = App.Gui.Selection.getSelection()
        stockcircles = sgetlabelofselectedgroupwithproperties(self.sel, ["sketchplane", "meshobject", "stockcirclestype"])

        planwindings = sgetlabelofselectedgroupwithproperties(self.sel, ["stockcircles"])
        if planwindings:
            self.form.qplanwindings.setText(planwindings)
        planwindingsgroup = sfindobjectbylabel(self.doc, self.form.qplanwindings.text())

        if planwindingsgroup and not stockcircles:
            stockcircles = planwindingsgroup.stockcircles
        if stockcircles:
            self.form.qstockcircles.setText(stockcircles)
            
        stockcirclesgroup = sfindobjectbylabel(self.doc, self.form.qstockcircles.text())
        print("stockcirclesgroup", stockcircles)
        if stockcirclesgroup:
            self.form.qsketchplane.setText(stockcirclesgroup.sketchplane)
            self.form.qmeshobject.setText(stockcirclesgroup.meshobject)
            self.form.qalongwire.setValue(stockcirclesgroup.alongwire)
        if planwindingsgroup and len(planwindingsgroup.OutList):
            self.form.qcurrentsplayangle.setValue(planwindingsgroup.OutList[-1].splayangle)

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
        print("Make onion layers mesh")
        planwindingsgroup = sfindobjectbylabel(self.doc, self.form.qplanwindings.text())
        onionlayers = self.form.qonionlayers.text()
        onionlayersgroup = freecadutils.getemptyfolder(self.doc, onionlayers)
        previewmultiplier = float(self.form.qpreviewmultiplier.text())

        stockcirclesfolder = sfindobjectbylabel(self.doc, self.form.qstockcircles.text())
        basethicknesses = [ 0.0 ] * len(stockcirclesfolder.OutList)

        for j, planwinding in enumerate(planwindingsgroup.OutList):
            for i in range(len(planwindingsgroup.OutList)):
                basethicknesses[i] += planwinding.circlethicknesses[i]*planwinding.plannedwinds
        
            additionalthicknesses = [ basecirclethickness + circlethickness*planwinding.plannedwinds \
                        for basecirclethickness, circlethickness in zip(basethicknesses, planwinding.circlethicknesses) ]
    
            ptpairs = [ (stockcircle.pt + stockcircle.norm*(basethickness*previewmultiplier), stockcircle.pt + stockcircle.norm*(additionalthickness*previewmultiplier))  \
                    for stockcircle, basethickness, additionalthickness in zip(stockcirclesfolder.OutList, basethicknesses, additionalthicknesses) ]
            facets = [ ]
            for i in range(len(ptpairs)-1):
                p0, p1 = ptpairs[i]
                pe0, pe1 = ptpairs[i+1]
                facets.append([p0, pe0, p1])
                facets.append([p1, pe0, pe1])
            onionlayername = "splang%.01f" % planwinding.splayangle
            onionlayer = freecadutils.doc.addObject("Mesh::Feature", onionlayername)
            onionlayer.Mesh = Mesh.Mesh(facets)
            onionlayer.ViewObject.Lighting = "Two side"
            onionlayer.ViewObject.ShapeColor = (float(planwinding.splayangle)/90.0, 0.7, 0.5 + 0.4*(j%2))
            onionlayersgroup.addObject(onionlayer)
            basethicknesses = additionalthicknesses

        
    def apply(self):
        print("apply!!")
        stockcirclesfolder = sfindobjectbylabel(self.doc, self.form.qstockcircles.text())
        alongwire = float(self.form.qalongwire.text())
        thinningtol = float(self.form.qthinningtol.text())
        towrad = float(self.form.qtowwidth.text())/2.0
        towthickness = float(self.form.qtowthickness.text())
        desiredthickness = float(self.form.qdesiredthickness.text())
        desiredthicknesslower = float(self.form.qdesiredthicknesslower.text())
        minimalwinds = int(self.form.qminimalwinds.text())
        planwindingsgroup = sfindobjectbylabel(self.doc, self.form.qplanwindings.text())
        if not planwindingsgroup:
            planwindingsgroup = freecadutils.getemptyfolder(self.doc, self.form.qplanwindings.text())
        setpropertyval(planwindingsgroup, "App::PropertyString", "stockcircles", stockcirclesfolder.Label)

        currentsplayangle = float(self.form.qcurrentsplayangle.text())
        endsplayangle = float(self.form.qendsplayangle.text())
        stepsplayangle = float(self.form.qstepsplayangle.text())
        stepsplayangle = abs(stepsplayangle)*(1 if endsplayangle >= currentsplayangle else -1)

        
        circlepaths = MandrelPaths(wirestopatharrays(stockcirclesfolder.OutList), towrad)
        assert len(circlepaths.mandrelptpaths) == len(stockcirclesfolder.OutList)
        previewmultiplier = float(self.form.qpreviewmultiplier.text())
        assert len(circlepaths.mandrelptpaths) == len(stockcirclesfolder.OutList)
        circlelengths = [ circlepaths.getgaplength(i*circlepaths.Nm + 0.0, i*circlepaths.Nm + len(circlepaths.mandrelptpaths[i])-1) \
                          for i in range(len(stockcirclesfolder.OutList)) ]

        # we are going to sum these up from planwindingsgroup
        basethicknesses = [ 0.0 ] * len(circlelengths)
        for planwinding in planwindingsgroup.OutList:
            for i in range(len(circlelengths)):
                basethicknesses[i] += planwinding.circlethicknesses[i]*planwinding.plannedwinds
        
        while True:
            currentsplayangle += stepsplayangle
            print("currentsplayangle", currentsplayangle)
            if (currentsplayangle > endsplayangle) == (stepsplayangle > 0):
                print("No more splay angles left")
                return
            self.form.qcurrentsplayangle.setValue(currentsplayangle)
            gbStart = self.drivecurve.startalongangle(alongwire, currentsplayangle)
            gbs = drivegeodesicRI(gbStart, self.drivecurve.drivebars, self.drivecurve.tridrivebarsmap, LRdirection=LRdirection, sideslipturningfactor=0, maxlength=maxlength)
            if gbs[-1] == None:
                continue
            alongwirelanded, angcrosslanded = self.drivecurve.endalongpositionA(gbs[-1])
            name = 'w%.1f' % currentsplayangle
            splayhooppts = [ gb.pt  for gb in gbs ]
            circlethicknesses = calccirclethicknesses(splayhooppts, circlepaths, circlelengths, towrad, towthickness)
            minwindstothickness = min((desiredthickness - basethickness)/circlethickness  \
                    for circlethickness, basethickness in zip(circlethicknesses, basethicknesses)  if circlethickness != 0)
            print(minwindstothickness, max(circlethicknesses))
            if minwindstothickness < minimalwinds:
                continue

            plannedwinds = int(minwindstothickness)
            planwinding = Part.show(Part.makePolygon([Vector(*p)  for p in splayhooppts]), name)
            setpropertyval(planwinding, "App::PropertyInteger", "plannedwinds", plannedwinds)
            setpropertyval(planwinding, "App::PropertyFloat", "alongwire", alongwire)
            setpropertyval(planwinding, "App::PropertyAngle", "splayangle", currentsplayangle)
            setpropertyval(planwinding, "App::PropertyFloat", "alongwirelanded", alongwirelanded)
            setpropertyval(planwinding, "App::PropertyAngle", "angcrosslanded", angcrosslanded)
            setpropertyval(planwinding, "App::PropertyFloatList", "circlethicknesses", circlethicknesses)
            planwindingsgroup.addObject(planwinding)
            for i in range(len(circlelengths)):
                basethicknesses[i] += planwinding.circlethicknesses[i]*planwinding.plannedwinds
            break
        return
            
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

        Dsplaycirclerad = max(v.X  for v in splaycircle.Shape.Vertexes)
        print(Dsplaycirclerad, towrad, (Dsplaycirclerad - towrad)/Dsplaycirclerad)
        Dcircleproportioncover = math.degrees(math.acos((Dsplaycirclerad - towrad)/Dsplaycirclerad))*2/360
        DcircleproportioncoverFarEdge = math.degrees(math.acos((Dsplaycirclerad - towrad)/(Dsplaycirclerad + towrad)))*2/360
        print("additionalthicknessperwindingProp", additionalthicknessperwinding/towthickness, "predicted", Dcircleproportioncover, "far edge bigger by", DcircleproportioncoverFarEdge/Dcircleproportioncover)

        requiredadditionalthickness = desiredthickness - meanbasethickness
        requiredwindingsforthickness = int(requiredadditionalthickness/additionalthicknessperwinding) + 1
        adjustedalongwirelanded, adjustedwindings = adjustlandingrepeatstobecoprime(splaycircle.alongwire, splaycircle.alongwirelanded, requiredwindingsforthickness)

        Dadustedalongwireadvance = adjustedalongwirelanded - alongwire
        if Dadustedalongwireadvance <= 0.0:
            Dadustedalongwireadvance += 1.0
        print("turns", round(Dadustedalongwireadvance*adjustedwindings), "for windings", adjustedwindings)

 
        gbs, fLRdirection, dseg, alongwirelanded = directedgeodesic(combofoldbackmode, self.drivecurve, self.utbm, alongwire, adjustedalongwirelanded, float(splaycircle.dsangle), Maxsideslipturningfactor, mandrelradius, 0.0, maxlength, None)
        rpts = repeatwindingpath([P3(*gb.pt)  for gb in gbs], adjustedwindings, thinningtol)
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

        #re = envelope radius
        #tr = towwidth/2
        #r = radius to make measurement
        #acos(min(1,(re-tr)/r))/pi - acos(min(1,(re+tr)/r))/pi


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

Gui.Control.showDialog(GenConstThickOnStockCirclesTaskPanel())