# -*- coding: utf-8 -*-

# directional geodesics from embedded curve controlling endpoint

# Embed a curve into a mesh so we can head off in different directions and tell when it is crossed

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 
from PySide import QtGui, QtCore

import os, sys, math, time
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])

import curvesutils;  import sys;  sys.modules.pop("curvesutils")
import trianglemeshutils;  import sys;  sys.modules.pop("trianglemeshutils")
import geodesicutils;  import sys;  sys.modules.pop("geodesicutils")
import freecadutils;  import sys;  sys.modules.pop("freecadutils")

from barmesh.basicgeo import I1, Partition1, P3, P2, Along
from curvesutils import isdiscretizableobject, discretizeobject, thinptstotolerance
from curvesutils import cumlengthlist, seglampos
from trianglemeshutils import UsefulBoxedTriangleMesh, facetbetweenbars
from wireembeddingutils import planecutembeddedcurve, planecutbars

from geodesicutils import drivegeodesic, InvAlong, GBarT, GBarC, drivecurveintersectionfinder, trilinecrossing, TOL_ZERO

import freecadutils
freecadutils.init(App)


def okaypressed():
    print("Okay Pressed") 
    singlewindobject = freecadutils.findobjectbylabel(qsinglewindpath.text())
    mandrelwindings = int(qmandrelwindings.text())
    mandrelwindingsmultiples = int(qmandrelwindingsmultiples.text())
    thintol = float(qthintol.text())
    
    singlewindpts = [ P3(p.X, p.Y, p.Z)  for p in singlewindobject.Shape.Vertexes ]
    fvec0 = P2(singlewindpts[0].x, singlewindpts[0].z)
    fvec1 = P2(singlewindpts[-1].x, singlewindpts[-1].z)
    print(singlewindpts[0].y, singlewindpts[-1].y, fvec0.Len(), fvec1.Len())
    angadvance = P2(P2.Dot(fvec0, fvec1), P2.Dot(fvec0, P2.APerp(fvec1))).Arg()

    tpt0 = thinptstotolerance(singlewindpts, tol=thintol)
    tpt = tpt0[:]
    for i in range(1, mandrelwindings*mandrelwindingsmultiples):
        rotcos = math.cos(math.radians(i*angadvance))
        rotsin = math.sin(math.radians(i*angadvance))
        for pt in tpt0[1:]:
            tpt.append(P3(pt.x*rotcos + pt.z*rotsin, pt.y, pt.z*rotcos - pt.x*rotsin))
    Part.show(Part.makePolygon([Vector(pt)  for pt in tpt]), qoutputfilament.text())
    qw.hide()

mandrelwindings = 10
mandrelrotations = 7
qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 500, 570, 350)
qw.setWindowTitle('Repeat winding toolpath')
qsinglewindpath = freecadutils.qrow(qw, "Single wind: ", 15+35*1)
mandrelwindings = 10


qmandrelwindings = freecadutils.qrow(qw, "Windings adv: ", 15+35*2, "%d" % mandrelwindings)
qmandrelwindingsmultiples = freecadutils.qrow(qw, "Windings mult: ", 15+35*3, "2")
qoutputfilament = freecadutils.qrow(qw, "Output name: ", 15+35*4, "t1")
qthintol = freecadutils.qrow(qw, "Thinning tol: ", 15+35*5, "0.2")

okButton = QtGui.QPushButton("Repeat", qw)
okButton.move(180, 15+35*7)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  

qsinglewindpath.setText(freecadutils.getlabelofselectedwire())

qw.show()


