# -*- coding: utf-8 -*-

# stock viewer task, a standard interactive panel on the left in FreeCAD.  
# To edit the UI file, nix-shell -p qtcreator, start up qtcreator
# select default current project and start editing stockviewertask.ui


import Draft, Part, Mesh, MeshPart, Fem
from FreeCAD import Vector, Rotation 
from PySide import QtGui, QtCore


import os, sys, math, time, numpy, re
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


def wirestopatharrays(wirelist):
    basepts = [ ]
    for bw in wirelist:
        if hasattr(bw, "Shape") and isinstance(bw.Shape, Part.Wire):
            basepts.append([P3(v.X,v.Y,v.Z)  for v in bw.Shape.Vertexes])
    return basepts

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

def makehoopthicknessmesh(stockcircles, circlethicknessesLo, circlethicknessesHi, multiplier):
    ptpairs = [ (stockcircle.pt + stockcircle.norm*(circlethicknessLo*multiplier), stockcircle.pt + stockcircle.norm*(circlethicknessHi*multiplier))  \
                for stockcircle, circlethicknessLo, circlethicknessHi in zip(stockcircles, circlethicknessesLo, circlethicknessesHi) ]
    facets = [ ]
    for i in range(len(ptpairs)-1):
        p0, p1 = ptpairs[i]
        pe0, pe1 = ptpairs[i+1]
        facets.append([p0, pe0, p1])
        facets.append([p1, pe0, pe1])
    return Mesh.Mesh(facets)


def calccirclethicknesses(splayhoop, circlepaths, circlelengths, towrad, towthickness):
    splayhooppts = [P3(v.X,v.Y,v.Z)  for v in splayhoop.Shape.Vertexes]

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
    

class GenConstThickFromSplayTaskPanel(QtGui.QWidget):
    def __init__(self):
        x = os.path.join(os.path.split(__file__)[0], "stockhoopmeasuretask.ui")
        self.form = FreeCADGui.PySideUic.loadUi(x)
        self.form.setMinimumSize(0, 420)
        QtCore.QObject.connect(self.form.buttontest, QtCore.SIGNAL("pressed()"), self.testbutton)
        self.update()

    def update(self):
        self.doc = App.ActiveDocument
        self.sel = App.Gui.Selection.getSelection()
        stockcircles = sgetlabelofselectedgroupwithproperties(self.sel, ["stockcirclestype"])
        if stockcircles:
            self.form.qstockcircles.setText(stockcircles)
        splayhoops = sgetlabelofselectedgroupwithproperties(self.sel, ["splayhooptype"])
        if splayhoops:
            self.form.qsplayhoops.setText(splayhoops)
        stockcirclesgroup = sfindobjectbylabel(self.doc, self.form.qstockcircles.text())
        if stockcirclesgroup:
            self.form.qsketchplane.setText(stockcirclesgroup.sketchplane)
            self.form.qmeshobject.setText(stockcirclesgroup.meshobject)
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
        towrad = float(self.form.qtowwidth.text())/2.0
        towthickness = float(self.form.qtowthickness.text())
        stockcirclesfolder = sfindobjectbylabel(self.doc, self.form.qstockcircles.text())
        splayhoopsfolder = sfindobjectbylabel(self.doc, self.form.qsplayhoops.text())
        circlepaths = MandrelPaths(wirestopatharrays(stockcirclesfolder.OutList), towrad)
        assert len(circlepaths.mandrelptpaths) == len(stockcirclesfolder.OutList)
        multiplier = float(self.form.qmultiplier.text())
        
        splayhoopindexes = list(map(int, re.findall("\d+", self.form.qsplayhoopindexes.text())))
        assert len(circlepaths.mandrelptpaths) == len(stockcirclesfolder.OutList)
        circlelengths = [ circlepaths.getgaplength(i*circlepaths.Nm + 0.0, i*circlepaths.Nm + len(circlepaths.mandrelptpaths[i])-1) \
                          for i in range(len(stockcirclesfolder.OutList)) ]

        basecirclethicknesses = [ 0.0 ] * len(circlelengths)
        for i in splayhoopindexes:
            splayhoop = splayhoopsfolder.OutList[i]
            circlethicknesses = calccirclethicknesses(splayhoop, circlepaths, circlelengths, towrad, towthickness)
            additionalcirclethicknesses = [ a+b  for a, b in zip(basecirclethicknesses, circlethicknesses) ]

            stockmeshname = "stockmesh%d" % i
            stockmesh = freecadutils.doc.addObject("Mesh::Feature", stockmeshname)
            stockmesh.Mesh = makehoopthicknessmesh(stockcirclesfolder.OutList, basecirclethicknesses, additionalcirclethicknesses, multiplier)
            stockmesh.ViewObject.Lighting = "Two side"
            #stockmesh.ViewObject.DisplayMode = "Flat Lines"

            basecirclethicknesses = additionalcirclethicknesses

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
