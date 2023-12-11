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
from utils.pathutils import BallPathCloseRegions, MandrelPaths, MakeFEAcoloredmesh
import utils.freecadutils as freecadutils
freecadutils.init(App)
from utils.curvesutils import cumlengthlist, seglampos

import imp
import utils.directedgeodesic
imp.reload(utils.directedgeodesic)
from utils.directedgeodesic import directedgeodesic, makebicolouredwire, makedrivecurve, drivegeodesicRI, DriveCurve
from utils.trianglemeshutils import UsefulBoxedTriangleMesh
from utils.geodesicutils import drivegeodesic, InvAlong, GBarT, GBarC, drivecurveintersectionfinder, trilinecrossing, TOL_ZERO

import FreeCADGui as Gui
import PySide.QtGui as QtGui
import PySide.QtCore as QtCore


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


class GenConstThickFromSplayTaskPanel(QtGui.QWidget):
    def __init__(self):
        x = os.path.join(os.path.split(__file__)[0], "genconstthickfromsplaytask.ui")
        self.form = FreeCADGui.PySideUic.loadUi(x)
        self.form.setMinimumSize(0, 390)
        QtCore.QObject.connect(self.form.pushButton, QtCore.SIGNAL("pressed()"), self.pushbutton)  
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
            self.form.qsphradlimitnext.setValue(min(sc.sphrad  for sc in splaycirclesgroup.OutList[-10:]))

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


    def pushbutton(self):
        print("Pushbutton pressed")

    def apply(self):
        print("apply!!")
        splaycirclefolder = sfindobjectbylabel(self.doc, self.form.qsplaycircles.text())
        sketchplane = sfindobjectbylabel(self.doc, self.form.qsketchplane.text())
        meshobject = sfindobjectbylabel(self.doc, self.form.qmeshobject.text())
        alongwire = float(self.form.qalongwire.text())
        sphradlimitnext = float(self.form.qsphradlimitnext.text())
        outputwindingsgroup = sfindobjectbylabel(self.doc, self.form.qoutputwindings.text())
        if not outputwindingsgroup:
            outputwindingsgroup = freecadutils.getemptyfolder(self.doc, self.form.qoutputwindings.text())
            setpropertyval(outputwindingsgroup, "App::PropertyString", "splaycirclefolder", splaycirclefolder.Label)


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
