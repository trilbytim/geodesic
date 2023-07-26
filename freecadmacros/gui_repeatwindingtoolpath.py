# -*- coding: utf-8 -*-

# directional geodesics from embedded curve controlling endpoint

# Embed a curve into a mesh so we can head off in different directions and tell when it is crossed

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 
from PySide import QtGui, QtCore

import os, sys, math, time
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])

#import curvesutils;  import sys;  sys.modules.pop("curvesutils")
#import trianglemeshutils;  import sys;  sys.modules.pop("trianglemeshutils")
#import geodesicutils;  import sys;  sys.modules.pop("geodesicutils")
#import freecadutils;  import sys;  sys.modules.pop("freecadutils")

from barmesh.basicgeo import I1, Partition1, P3, P2, Along
from utils.curvesutils import isdiscretizableobject, discretizeobject, thinptstotolerance, cumlengthlist, seglampos
from utils.trianglemeshutils import UsefulBoxedTriangleMesh, facetbetweenbars
from utils.wireembeddingutils import planecutembeddedcurve, planecutbars

from utils.geodesicutils import drivegeodesic, InvAlong, GBarT, GBarC, drivecurveintersectionfinder, trilinecrossing, TOL_ZERO

import utils.freecadutils as freecadutils
freecadutils.init(App)


def okaypressed():
    print("Okay Pressed") 
    singlewindobjects = [ freecadutils.findobjectbylabel(singlewindname)  for singlewindname in qsinglewindpath.text().split(",") ]
    if qoutputfilament.text():
    	outputfilament = qoutputfilament.text()
    else:
    	outputfilament = qsinglewindpath.text() + 'x' + qmandrelwindings.text()

    mandrelwindings = int(qmandrelwindings.text())
    mandrelwindingsmultiples = int(qmandrelwindingsmultiples.text())
    thintol = float(qthintol.text())
    
    Ssinglewindpts = [ ]
    for singlewindobject in singlewindobjects:
        singlewindpts = [ P3(p.X, p.Y, p.Z)  for p in singlewindobject.Shape.Vertexes ]
        tsinglewindpts = thinptstotolerance(singlewindpts, tol=thintol)
        print("Thinned", len(singlewindpts), "points to", len(tsinglewindpts), "at tol", thintol)
        if len(Ssinglewindpts) != 0:
            sgapleng = (Ssinglewindpts[-1][-1] - tsinglewindpts[0]).Len()
            print("sgapleng", sgapleng)
            assert sgapleng < 0.01
            ptjoin = (Ssinglewindpts[-1][-1] + tsinglewindpts[0])*0.5
            Ssinglewindpts[-1][-1] = ptjoin
            tsinglewindpts[0] = ptjoin
        Ssinglewindpts.append(tsinglewindpts)
        
    ptfront, ptback = Ssinglewindpts[0][0], Ssinglewindpts[-1][-1]
    fvec0 = P2(ptfront.x, ptfront.z)
    fvec1 = P2(ptback.x, ptback.z)
    print("drive curve y-vals", ptfront.y, ptback.y, "rads", fvec0.Len(), fvec1.Len())
    angadvance = P2(P2.Dot(fvec0, fvec1), P2.Dot(fvec0, P2.APerp(fvec1))).Arg()

    tpt0 = Ssinglewindpts[0][:]
    for singlewindpts in Ssinglewindpts[1:]:
        tpt0.extend(singlewindpts[1:])
        
    tpt = tpt0[:]
    for i in range(1, mandrelwindings*mandrelwindingsmultiples):
        rotcos = math.cos(math.radians(i*angadvance))
        rotsin = math.sin(math.radians(i*angadvance))
        for pt in tpt0[1:]:
            tpt.append(P3(pt.x*rotcos + pt.z*rotsin, pt.y, pt.z*rotcos - pt.x*rotsin))
    Part.show(Part.makePolygon([Vector(pt)  for pt in tpt]), outputfilament)
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
qmandrelwindingsmultiples = freecadutils.qrow(qw, "Windings mult: ", 15+35*3, "1")
qoutputfilament = freecadutils.qrow(qw, "Output name: ", 15+35*4)
qthintol = freecadutils.qrow(qw, "Thinning tol: ", 15+35*5, "0.2")

okButton = QtGui.QPushButton("Repeat", qw)
okButton.move(180, 15+35*7)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  

qsinglewindpath.setText(freecadutils.getlabelofselectedwire(multiples=True))

qw.show()


