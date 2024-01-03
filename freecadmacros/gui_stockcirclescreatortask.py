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
    
def sfindobjectbylabel(doc, lab):
    objs = [ obj  for obj in doc.findObjects(Label=lab)  if obj.Label == lab ]
    return objs[0] if objs else None

def setpropertyval(obj, atype, name, value):
    if name not in obj.PropertiesList:
        obj.addProperty(atype, name, "filwind")
    setattr(obj, name, value)


mandrelradius = 110  # fc6 file
dsangle = 90.0
LRdirection = 1
appaturepapproachpoint = P3(0,-150,0)



def makestockcolumnline(gbts):
    stockcolumn = Part.show(Part.makePolygon([Vector(gbt.pt - gbt.tnorm*0)  for gbt in gbts]), "stockcolumn")
    #outputwindingsgroup.addObject(ply)
    #setpropertyval(stockcolumn, "App::PropertyAngle", "sphrad", splaycircle.sphrad)
    stockcolumn.addProperty("App::PropertyVectorList", "Normals")
    stockcolumn.Normals = [ Vector(-gbt.tnorm)  for gbt in gbts ] 


def chaseupcolumngbtsfromdrivecurveSingle(drivecurve, alongwire, colsamplestep):
    colgbStart = drivecurve.startalongangle(alongwire, dsangle)
    colgbs = drivegeodesicRI(colgbStart, drivecurve.drivebars, drivecurve.tridrivebarsmap, LRdirection=LRdirection, maxlength=400.0)
    if colgbs[-1] == None:
        colgbs.pop()

    colgbsT = [ None ] + [ (gb.bar, gb.lam)  for gb in colgbs[1:] ]
      # first element is a GBarT, which could be extended back to a GBarC but it's not an important area
      
    colcls = cumlengthlist([ gb.pt  for gb in colgbs ])

    gbts = [ ]
    ds = 0.0
    while ds < colcls[-1] and len(gbts) < 1000:
        dsseg, dslam = seglampos(ds, colcls)
        if dsseg != 0:
            gbt = GBarT(colgbsT, dsseg, dslam)
            gbts.append(gbt)
            #pt = Along(dslam, colgbs[dsseg].pt, colgbs[dsseg+1].pt)
            #norm = colgbs[dsseg+1].tnorm if hasattr(colgbs[dsseg+1], "tnorm") else colgbs[dsseg+1].tnorm_incoming
        ds += colsamplestep
    return gbts


class StockCirclesTaskPanel(QtGui.QWidget):
    def __init__(self):
        x = os.path.join(os.path.split(__file__)[0], "stockcirclescreatortask.ui")
        self.form = FreeCADGui.PySideUic.loadUi(x)
        self.form.setMinimumSize(0, 300)
        QtCore.QObject.connect(self.form.pushButton, QtCore.SIGNAL("pressed()"), self.pushbutton)  
        self.update()

    def update(self):
        self.doc = App.ActiveDocument
        self.sel = App.Gui.Selection.getSelection()
        self.form.qmeshobject.setText(sgetlabelofselectedmesh(self.sel))
        self.form.qsketchplane.setText(sgetlabelofselectedsketch(self.sel))

    def pushbutton(self):
        print("Pushbutton pressed")

    def apply(self):
        print("apply!!")
        sketchplane = sfindobjectbylabel(self.doc, self.form.qsketchplane.text())
        meshobject = sfindobjectbylabel(self.doc, self.form.qmeshobject.text())
        stockcirclesname = self.form.qstockcirclesname.text()
        alongwire = float(self.form.qalongwire.text())
        colsamplestep = float(self.form.qcolsamplestep.text())
        assert self.form.qappatureapproachpoint.text() == "P3(0,-150,0)"

        utbm = UsefulBoxedTriangleMesh(meshobject.Mesh)
        drivecurve = makedrivecurve(sketchplane, utbm, mandrelradius)

        gbts = chaseupcolumngbtsfromdrivecurveSingle(drivecurve, alongwire, colsamplestep)
        makestockcolumnline(gbts)
        
        stockcirclesgroup = freecadutils.getemptyfolder(self.doc, stockcirclesname)
        setpropertyval(stockcirclesgroup, "App::PropertyFloat", "alongwire", alongwire)
        setpropertyval(stockcirclesgroup, "App::PropertyString", "sketchplane", sketchplane.Label)
        setpropertyval(stockcirclesgroup, "App::PropertyString", "meshobject", meshobject.Label)
        setpropertyval(stockcirclesgroup, "App::PropertyBool", "stockcirclestype", True)


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

Gui.Control.showDialog(StockCirclesTaskPanel())
