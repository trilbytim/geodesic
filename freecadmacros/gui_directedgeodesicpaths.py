# -*- coding: utf-8 -*-

# directional geodesics from embedded curve controlling endpoint

# Embed a curve into a mesh so we can head off in different directions and tell when it is crossed

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 
from PySide import QtGui, QtCore
from barmesh.basicgeo import P2

import os, sys, math, time
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])


import curvesutils;  import sys;  sys.modules.pop("curvesutils")
import trianglemeshutils;  import sys;  sys.modules.pop("trianglemeshutils")
import geodesicutils;  import sys;  sys.modules.pop("geodesicutils")
import freecadutils;  import sys;  sys.modules.pop("freecadutils")

from barmesh.basicgeo import I1, Partition1, P3, P2, Along
from curvesutils import isdiscretizableobject, discretizeobject
from curvesutils import cumlengthlist, seglampos
from trianglemeshutils import UsefulBoxedTriangleMesh, facetbetweenbars
from wireembeddingutils import planecutembeddedcurve, planecutbars

from geodesicutils import drivegeodesic, InvAlong, GBarT, drivecurveintersectionfinder, trilinecrossing, TOL_ZERO

import freecadutils
freecadutils.init(App)

# could send this back into geodesicutils
def Idrivecurveintersectionfinder(drivebars, tridrivebarsmap, gb0, gb1, LRdirection=0):
    tbar = facetbetweenbars(gb0.bar, gb1.bar)
    if tbar.i not in tridrivebarsmap:
        return None
    dseg = tridrivebarsmap[tbar.i]
    Dtbar = facetbetweenbars(drivebars[dseg][0], drivebars[dseg+1][0])
    assert tbar == Dtbar
    dlam = trilinecrossing(tbar, drivebars[dseg], drivebars[dseg+1], (gb0.bar, gb0.lam), (gb1.bar, gb1.lam))
    if dlam == -1.0:
        return None
    res = GBarT(drivebars, dseg, dlam)
    if LRdirection != 0:
        tperpdorapproach = P3.Dot(res.tperp, gb1.pt - gb0.pt)
        if (tperpdorapproach > 0.0) == (LRdirection == 1):
            print("crossing point detected but from wrong side")
            return None
    res.dcseg = dseg
    res.dclam = dlam
    TOL_ZERO(P3.Cross(gb1.tnorm_incoming, res.tnorm).Len())
    res.tnorm_incoming = gb1.tnorm_incoming
    res.gbBackbarC = gb0
    res.gbForebarC = gb1
    return res

def drivesetBFstartfromangle(drivebars, dpts, dptcls, ds, dsangle):
    dsseg, dslam = seglampos(ds, dptcls)
    gbStart = GBarT(drivebars, dsseg, dslam)
    gb, gbbackbar = gbStart.drivepointstartfromangle(dsangle, getbackbar=True)
    gbStart.gbBackbarC = gbbackbar
    gbStart.gbForebarC = gb
    return gbStart


def drivegeodesicR(drivebars, tridrivebarsmap, dpts, dptcls, ds, dsangle, MAX_SEGMENTS=2000):
    gbStart = drivesetBFstartfromangle(drivebars, dpts, dptcls, ds, dsangle)
    gbs = [ gbStart, gbStart.gbForebarC ]
    Nconcavefolds = 0
    while True:
        gb = gbs[-1].GBCrossBar(gbs[-2].pt, None)
        if not gb or len(gbs) > MAX_SEGMENTS:
            print("exceeded MAX_SEGMENTS", MAX_SEGMENTS)
            return gbs, -1, -1
        gbEnd = Idrivecurveintersectionfinder(drivebars, tridrivebarsmap, gbs[-1], gb, LRdirection=1)
        if gbEnd:
            gbs.append(gbEnd)
            break
        gbs.append(gb)
    angcross = gbEnd.drivecurveanglefromvec(gbs[-1].pt - gbs[-2].pt)
    dcross = Along(gbEnd.dclam, dptcls[gbEnd.dcseg], dptcls[gbEnd.dcseg+1])
    return gbs, dcross, angcross

def drivegeodesicRI(gbStart, drivebars, tridrivebarsmap, LRdirection=1, MAX_SEGMENTS=2000):
    gbs = [ gbStart, gbStart.gbForebarC ]
    Nconcavefolds = 0
    while True:
        gbFore = gbs[-1].GBCrossBar(gbs[-2].pt, None)
        if not gbFore or len(gbs) > MAX_SEGMENTS:
            print("exceeded MAX_SEGMENTS or off edge", MAX_SEGMENTS)
            gbs.append(None)
            break
        gbEnd = Idrivecurveintersectionfinder(drivebars, tridrivebarsmap, gbs[-1], gbFore, LRdirection=LRdirection)
        if gbEnd:
            gbs.append(gbEnd)
            break
        gbs.append(gbFore)
    return gbs

# meshconvexificationbyflipedge

def okaypressed():
    print("Okay Pressed") 
    sketchplane = freecadutils.findobjectbylabel(qsketchplane.text())
    meshobject = freecadutils.findobjectbylabel(qmeshobject.text())
    alongwire = float(qalongwire.text())
    dsangle = float(qanglefilament.text())
    alongwireI = float(qalongwireI.text())
    dsangleI = float(qanglefilamentI.text())

    if not (sketchplane and meshobject):
        print("Need to select a Sketch and a Mesh object in the UI to make this work")
        qw.hide()
        return

    driveperpvec = sketchplane.Placement.Rotation.multVec(Vector(0,0,1))
    driveperpvecDot = driveperpvec.dot(sketchplane.Placement.Base)
    rotplanevecX = sketchplane.Placement.Rotation.multVec(Vector(1,0,0))
    rotplanevecY = sketchplane.Placement.Rotation.multVec(Vector(0,1,0))
    utbm = UsefulBoxedTriangleMesh(meshobject.Mesh)
    flatbartangents = None

    startbar, startlam = planecutbars(utbm.tbarmesh, driveperpvec, driveperpvecDot)
    drivebars = planecutembeddedcurve(startbar, startlam, driveperpvec)
    tridrivebarsmap = dict((facetbetweenbars(drivebars[dseg][0], drivebars[dseg+1][0]).i, dseg)  for dseg in range(len(drivebars)-1))
    dpts = [ Along(lam, bar.nodeback.p, bar.nodefore.p)  for bar, lam in drivebars ]
    dptcls = cumlengthlist(dpts)

    ds = Along(alongwire, dptcls[0], dptcls[-1])
    gbStart = drivesetBFstartfromangle(drivebars, dpts, dptcls, ds, dsangle)
    gbs = drivegeodesicRI(gbStart, drivebars, tridrivebarsmap)
    Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbs]), qoutputfilament.text())
    
    gbsS = gbs[:]
    gbsS[0] = gbsS[0].gbBackbarC
    gbsS[-1] = gbsS[-1].gbForebarC

    drivebarsB = [ (gb.bar, gb.lam)  for gb in gbsS ]
    tridrivebarsmapB = dict((facetbetweenbars(drivebarsB[dseg][0], drivebarsB[dseg+1][0]).i, dseg)  for dseg in range(len(drivebarsB)-1))
    dsI = Along(alongwireI, dptcls[0], dptcls[-1])
    gbStartI = drivesetBFstartfromangle(drivebars, dpts, dptcls, dsI, dsangleI+180)
    gbsI = drivegeodesicRI(gbStartI, drivebarsB, tridrivebarsmapB, LRdirection=0, MAX_SEGMENTS=100)
    Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in (gbsI[:-1] if gbsI[-1] is None else gbsI)]), qoutputfilament.text())

    gbEnd = gbs[-1]
    if gbEnd is not None:
        dsangleForeIn = gbEnd.drivecurveanglefromvec(gbEnd.gbForebarC.pt - gbEnd.gbBackbarC.pt)
        dsForeIn = Along(gbEnd.dclam, dptcls[gbEnd.dcseg], dptcls[gbEnd.dcseg+1])
        print("pos dsForeIn", dsForeIn, dsangleForeIn)
        print(windingangle(gbs, rotplanevecX, rotplanevecY))
        qalongwire.setText("%f" % InvAlong(dsForeIn, dptcls[0], dptcls[-1]))
        qanglefilament.setText("%f" % dsangleForeIn)
        print("Cylinder position angle advance degrees", 360*(dsForeIn - ds)/dptcls[-1])
        print("Leaving angle", dsangle, "Continuing angle", dsangleForeIn)
        try:
            qoutputfilament.setText("w%d" % (int(qoutputfilament.text()[1:]) + 1))
        except ValueError:
            pass

def windingangle(gbs, rotplanevecX, rotplanevecY):
    prevFV = None
    sumA = 0.0
    for gb in gbs:
        FV = P2(P3.Dot(rotplanevecX, gb.pt), P3.Dot(rotplanevecY, gb.pt))
        if prevFV is not None:
            dvFA = P2(P2.Dot(prevFV, FV), P2.Dot(P2.APerp(prevFV), FV)).Arg()
            sumA += dvFA
        prevFV = FV
    return sumA

qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 500, 570, 350)
qw.setWindowTitle('Drive geodesic')
qsketchplane = freecadutils.qrow(qw, "Sketchplane: ", 15+35*0)
qmeshobject = freecadutils.qrow(qw, "Meshobject: ", 15+35*1 )

qalongwire = freecadutils.qrow(qw, "Along wire: ", 15+35*2, "0.51")
qanglefilament = freecadutils.qrow(qw, "Angle filament: ", 15+35*3, "30.0")
qalongwireI = freecadutils.qrow(qw, "Along wire in: ", 15+35*2, "0.28", 260) # approx 0.304
qanglefilamentI = freecadutils.qrow(qw, "Angle fil. in: ", 15+35*3, "30.0", 260)

qoutputfilament = freecadutils.qrow(qw, "Output name: ", 15+35*4, "w1")
okButton = QtGui.QPushButton("Drive", qw)
okButton.move(180, 15+35*7)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  

qsketchplane.setText(freecadutils.getlabelofselectedsketch())
qmeshobject.setText(freecadutils.getlabelofselectedmesh())

qw.show()


