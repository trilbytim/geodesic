# -*- coding: utf-8 -*-

# directional geodesics from embedded curve controlling endpoint

# Embed a curve into a mesh so we can head off in different directions and tell when it is crossed

import Draft, Part, Mesh, MeshPart, Fem
from FreeCAD import Vector, Rotation 
from PySide import QtGui, QtCore


import os, sys, math, time
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])


from barmesh.basicgeo import I1, Partition1, P3, P2, Along, lI1
from barmesh.tribarmes.triangleboxing import TriangleBoxing
from utils.pathutils import BallPathCloseRegions, MandrelPaths, MakeFEAcoloredmesh

import utils.freecadutils as freecadutils
freecadutils.init(App)



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
