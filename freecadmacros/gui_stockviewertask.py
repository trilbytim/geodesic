# -*- coding: utf-8 -*-

# directional geodesics from embedded curve controlling endpoint

# Embed a curve into a mesh so we can head off in different directions and tell when it is crossed

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

import imp
import utils.directedgeodesic
imp.reload(utils.directedgeodesic)
from utils.directedgeodesic import directedgeodesic, makebicolouredwire, makedrivecurve, drivegeodesicRI, DriveCurve
from utils.trianglemeshutils import UsefulBoxedTriangleMesh

def okaypressed():
    print("Okay Pressed") 
    mandrelpaths = [ freecadutils.findobjectbylabel(mandrelpathname)  for mandrelpathname in qmandrelpaths.text().split(",") ]
    towwidth = float(qtowwidth.text())/2
    towthick = float(qtowthick.text())
    
    measuremesh = freecadutils.findobjectbylabel(qmeshpointstomeasure.text())
    if measuremesh.TypeId == "Mesh::Curvature":
        meshcurvature = measuremesh
        measuremesh = meshcurvature.Source
    else:
        meshcurvature = None
    
    mandrelptpaths = [ ]
    for mandrelpath in mandrelpaths:
        mandrelwindpts = [ P3(p.X, p.Y, p.Z)  for p in mandrelpath.Shape.Vertexes ]
        mandrelptpaths.append(mandrelwindpts)
    mandpaths = MandrelPaths(mandrelptpaths)
    xrg = mandpaths.xrg.Inflate(towwidth*2)
    yrg = mandpaths.yrg.Inflate(towwidth*2)
    boxwidth = max(towwidth, xrg.Leng()/30, yrg.Leng()/30)
    tbs = TriangleBoxing(None, xrg.lo, xrg.hi, yrg.lo, yrg.hi, boxwidth)
    print("Creating box set boxwidth=", boxwidth, mandpaths.Nm)
    mandpaths.addpathstotgbs(tbs)

    thickcount = [ ]
    maxthickcount = 0
    thickpoint = None
    for mp in measuremesh.Mesh.Points[::]:
        mmp = P3(mp.x, mp.y, mp.z)
        bpcr = BallPathCloseRegions(mmp, towwidth)
        mandpaths.nhitreg += 1
        for ix, iy in tbs.CloseBoxeGenerator(mp.x, mp.x, mp.y, mp.y, towwidth):
            tbox = tbs.boxes[ix][iy]
            for i in tbox.pointis:
                bpcr.DistPoint(mandpaths.getpt(i), i)
            for i in tbox.edgeis:
                if mandpaths.hitreg[i] != mandpaths.nhitreg:
                    bpcr.DistEdge(mandpaths.getpt(i), mandpaths.getpt(i+1), i)
                    mandpaths.hitreg[i] = mandpaths.nhitreg
        bpcr.mergeranges()
        #ss = len(bpcr.ranges)
        #bpcr.mergegaps(0.1, mandpaths)
        #if ss != len(bpcr.ranges):
        #    print("Gap actually merged")
        thickcount.append(len(bpcr.ranges))
        if thickcount[-1] > maxthickcount:
            maxthickcount = thickcount[-1]
            thickpoint = mp
        
    print("Max thick count", maxthickcount, "thickness", maxthickcount*towthick, "at point", thickpoint)
    if meshcurvature != None:
        for i, c in enumerate(thickcount):
            meshcurvature.ValueAtIndex = (i, c*towthick, c)
            meshcurvature.recompute()
        print(" Setting of Min/Max curvatures to filament crossings")
    else:
        if "VertexThicknesses" not in measuremesh.PropertiesList:
            measuremesh.addProperty("App::PropertyFloatList", "VertexThicknesses")
        measuremesh.VertexThicknesses = [ c*towthick  for c in thickcount ]
    qw.hide()

if False:
    qw = QtGui.QWidget()
    qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
    qw.setGeometry(700, 500, 300, 350)
    qw.setWindowTitle('Measure thickness')
    qmeshpointstomeasure = freecadutils.qrow(qw, "Mesh: ", 15+35*1)

    qmandrelpaths = freecadutils.qrow(qw, "Winding paths ", 15+35*2, "")
    qtowwidth = freecadutils.qrow(qw, "Tow width: ", 15+35*3, "6.35")
    qtowthick = freecadutils.qrow(qw, "Tow thick: ", 15+35*4, "0.18")

    okButton = QtGui.QPushButton("Measure", qw)
    okButton.move(180, 15+35*8)
    QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  

    qmandrelpaths.setText(freecadutils.getlabelofselectedwire(multiples=True))
    qmeshpointstomeasure.setText(freecadutils.getlabelofselectedmesh())

    qw.show()






# Should be put back into the useful utils
def sgetlabelsofselectedwire(sel, multiples=False):
    labels = [ ]
    for s in sel:
        if hasattr(s, "Shape") and isinstance(s.Shape, Part.Wire):
            labels.append(s.Label)
            if not multiples:
                break
    return ",".join(labels)

def sgetlabelstowwires(sel):
    labels = [ ]
    lsel = sel.copy()
    while lsel:
        s = lsel.pop()
        if hasattr(s, "Shape") and isinstance(s.Shape, Part.Wire):
            if "towwidth" in s.PropertiesList and not (s.Label in labels):
                labels.append(s.Label)
        elif hasattr(s, "OutList"):
            lsel.extend(s.OutList)
    return ",".join(labels)

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



import FreeCADGui as Gui
import PySide.QtGui as QtGui
import PySide.QtCore as QtCore

Maxsideslipturningfactor = 0.26
combofoldbackmode = 0
mandrelradius = 110  # fc6 file


class CCCC(QtGui.QWidget):
    def __init__(self):
        x = os.path.join(os.path.split(__file__)[0], "stockviewertask.ui")
        self.form = FreeCADGui.PySideUic.loadUi(x)
        self.form.setMinimumSize(0, 300)
        QtCore.QObject.connect(self.form.pushButton, QtCore.SIGNAL("pressed()"), self.pushbutton)  
        self.update()

    def update(self):
        self.doc = App.ActiveDocument
        self.sel = App.Gui.Selection.getSelection()
        self.form.qmandrelpaths.setText(sgetlabelstowwires(self.sel))
        self.form.qmeshobject.setText(sgetlabelofselectedmesh(self.sel))
        self.form.qsketchplane.setText(sgetlabelofselectedsketch(self.sel))

    def pushbutton(self):
        print("Pushbutton pressed")

    def apply(self):
        print("apply!")
        sketchplane = sfindobjectbylabel(self.doc, self.form.qsketchplane.text())
        meshobject = sfindobjectbylabel(self.doc, self.form.qmeshobject.text())
        alongwireLo = float(self.form.qalongwireLo.text())
        alongwireHi = float(self.form.qalongwireHi.text())
        columns = int(self.form.qcolumns.text())
        utbm = UsefulBoxedTriangleMesh(meshobject.Mesh)
        drivecurve = makedrivecurve(sketchplane, utbm, mandrelradius)


# We are making a radial path that can be a secondary drive curve
# that we will need to rotate around the axis
# And use this to define the underlying stock mesh
# which will lie in a sector and have its own normal vectors embedded
# and we can use this to project outward for the stock meshes.
# call this build stock basis mesh
        dsangle, LRdirection = 90.0, 1
        perpdrivecurves = [ ]
        for alongwire in numpy.linspace(alongwireLo, alongwireHi, columns+1):
            gbStart = drivecurve.startalongangle(alongwire, dsangle)
            gbs = drivegeodesicRI(gbStart, drivecurve.drivebars, drivecurve.tridrivebarsmap, LRdirection=LRdirection, maxlength=400.0)
            gbdrivebars = [(gb.bar, gb.lam)  for gb in reversed(gbs[1:-2])]
            r = DriveCurve(gbdrivebars)  # this can't handle running to the edge case, so why it's -2, or the GbarT case
            perpdrivecurves.append(r)
            Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbs[:-1]]), "thing")

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

Gui.Control.showDialog(CCCC())

