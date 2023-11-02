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







def sgetlabelstowwires(sel):
    labels = [ ]
    lsel = sel.copy()
    while lsel:
        s = lsel.pop()
        if hasattr(s, "Shape") and isinstance(s.Shape, Part.Wire):
            if "towwidth" in s.PropertiesList and not (s.Label in labels):
                labels.append(s.Label)
            else:
                if not (hasattr(s, "Module") and s.Module == 'Sketcher'):
                    print("Warning: %s has no towwidth property" % s.Label)
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


mandrelradius = 110  # fc6 file
dsangle = 90.0
LRdirection = 1

def chaseupcolumnptnormalsfromdrivecurve(drivecurve, alongwireLo, alongwireHi, columns, colsamplestep):
    columnptnormals = [ ]
    for alongwire in numpy.linspace(alongwireLo, alongwireHi, columns+1):
        colgbStart = drivecurve.startalongangle(alongwire, dsangle)
        colgbs = drivegeodesicRI(colgbStart, drivecurve.drivebars, drivecurve.tridrivebarsmap, LRdirection=LRdirection, maxlength=400.0)
        if colgbs[-1] == None:
            colgbs.pop()

        colcls = cumlengthlist([gb.pt  for gb in colgbs])
        ds = 0.0
        ptnormals = [ ]
        while ds < colcls[-1] and len(ptnormals) < 100:
            dsseg, dslam = seglampos(ds, colcls)
            pt = Along(dslam, colgbs[dsseg].pt, colgbs[dsseg+1].pt)
            norm = colgbs[dsseg+1].tnorm if hasattr(colgbs[dsseg+1], "tnorm") else colgbs[dsseg+1].tnorm_incoming
            ptnormals.append((pt, -norm))
            ds += colsamplestep
        columnptnormals.append(ptnormals)
    return columnptnormals

def makemeshfromcolumnpts(columnpts):
    facets = [ ]
    for j in range(len(columnpts)-1):
        c0, c1 = columnpts[j], columnpts[j+1]
        for i in range(min(len(c0), len(c1))-1):
            p0, p1 = c0[i], c0[i+1]
            pe0, pe1 = c1[i], c1[i+1]
            facets.append([p0, pe0, p1])
            facets.append([p1, pe0, pe1])
    mesh = freecadutils.doc.addObject("Mesh::Feature", "stockshape")
    mesh.ViewObject.Lighting = "Two side"
    mesh.ViewObject.DisplayMode = "Flat Lines"
    mesh.Mesh = Mesh.Mesh(facets)


import FreeCADGui as Gui
import PySide.QtGui as QtGui
import PySide.QtCore as QtCore


class StockViewerTaskPanel(QtGui.QWidget):
    def __init__(self):
        print("loading stockviewertask.ui")
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
        print("apply!!")
        sketchplane = sfindobjectbylabel(self.doc, self.form.qsketchplane.text())
        meshobject = sfindobjectbylabel(self.doc, self.form.qmeshobject.text())
        alongwireLo = float(self.form.qalongwireLo.text())
        alongwireHi = float(self.form.qalongwireHi.text())
        columns = int(self.form.qcolumns.text())
        colsamplestep = float(self.form.qcolsamplestep.text())

        utbm = UsefulBoxedTriangleMesh(meshobject.Mesh)
        drivecurve = makedrivecurve(sketchplane, utbm, mandrelradius)

        columnptnormals = chaseupcolumnptnormalsfromdrivecurve(drivecurve, alongwireLo, alongwireHi, columns, colsamplestep)
        #Part.show(Part.makePolygon([Vector(*pt)  for pt, norm in ptnormals]), "thing")

        columnpts = [ [ pt+norm*4.1  for pt, norm in ptnormals ]  for ptnormals in columnptnormals ]
        makemeshfromcolumnpts(columnpts)


# We are making a radial path that can be a secondary drive curve
# that we will need to rotate around the axis
# And use this to define the underlying stock mesh
# which will lie in a sector and have its own normal vectors embedded
# and we can use this to project outward for the stock meshes.
# call this build stock basis mesh


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

Gui.Control.showDialog(StockViewerTaskPanel())

