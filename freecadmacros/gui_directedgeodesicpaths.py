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

from geodesicutils import drivegeodesic, InvAlong, GBarT, GBarC, drivecurveintersectionfinder, trilinecrossing, TOL_ZERO

import freecadutils
freecadutils.init(App)


def calcfriccoeff(pullforce, vn):
    alongforce = P3.Dot(pullforce, vn)
    sideforcesq = pullforce.Lensq() - alongforce*alongforce
    assert sideforcesq >= -0.001
    sideforce = math.sqrt(max(0, sideforcesq))
    return alongforce/sideforce

def calcfriction(gb, pt0, pt1):
    vn = P3.ZNorm(gb.bar.nodefore.p - gb.bar.nodeback.p)
    pullforce = P3.ZNorm(gb.pt - pt0) + P3.ZNorm(gb.pt - pt1)
    return calcfriccoeff(pullforce, vn)

def calcfriccoeff(pullforce, vn):
    alongforce = P3.Dot(pullforce, vn)
    sideforcesq = pullforce.Lensq() - alongforce*alongforce
    assert sideforcesq >= -0.001
    sideforce = math.sqrt(max(0, sideforcesq))
    return alongforce/sideforce

def calcfriccoeffbarEnds(pullforceFrom, vn):
    alongforce = P3.Dot(pullforceFrom, vn)
    sideforcesq = pullforceFrom.Lensq() - alongforce*alongforce
    assert sideforcesq >= -0.001
    sideforce = math.sqrt(max(0, sideforcesq))
    return (alongforce + 1.0)/sideforce, (alongforce - 1.0)/sideforce



def GBCrossBarRS(gb, ptpushfrom, sideslipturningfactor):
    v = gb.bar.nodefore.p - gb.bar.nodeback.p
    vn = P3.ZNorm(v)
    pullforceFrom = P3.ZNorm(gb.pt - ptpushfrom)

    sinalpha = P3.Dot(vn, pullforceFrom)
    cosalpha = math.sqrt(max(0.0, 1.0 - sinalpha**2))

    barforerightBL = gb.bar.GetForeRightBL(gb.bGoRight)
    if barforerightBL is None:
        return None
    tnodeopposite = barforerightBL.GetNodeFore(barforerightBL.nodeback == gb.bar.GetNodeFore(gb.bGoRight))

    trisidenorm = P3.ZNorm(P3.Cross(tnodeopposite.p - gb.bar.nodeback.p, v))
    trisideperp = P3.Cross(vn, trisidenorm)
    assert P3.Dot(trisideperp, tnodeopposite.p - gb.pt) >= 0.0
    
    fromGoRight = gb.bGoRight
    
    if sideslipturningfactor != 0.0:
        barforerightBLBack = gb.bar.GetForeRightBL(not gb.bGoRight)
        tnodeoppositeBack = barforerightBLBack.GetNodeFore(barforerightBLBack.nodeback == gb.bar.GetNodeFore(not gb.bGoRight))
        trisidenormBack = P3.ZNorm(P3.Cross(tnodeoppositeBack.p - gb.bar.nodeback.p, v))
        costheta = -P3.Dot(trisidenorm, trisidenormBack)
        tfoldangle = math.acos(costheta)
        siderotangle = sideslipturningfactor*tfoldangle*(-1 if gb.bGoRight else 1)
        sinra = math.sin(siderotangle)
        cosra = math.cos(siderotangle)
        sinbeta = sinalpha*cosra + cosalpha*sinra
        cosbeta = cosalpha*cosra - sinalpha*sinra
        TOL_ZERO(math.hypot(sinbeta, cosbeta) - 1.0)
        if cosbeta < 0.0:
            print("bouncing back from glancing edge")
            cosbeta = -cosbeta
            fromGoRight = not gb.bGoRight
            tnodeopposite = tnodeoppositeBack
            trisidenorm = trisidenormBack
            trisideperp = P3.Cross(vn, trisidenorm)
            assert P3.Dot(trisideperp, tnodeopposite.p - gb.pt) >= 0.0

    else:
        sinbeta = sinalpha
        cosbeta = cosalpha

    vecoppoutto = vn*sinbeta + trisideperp*cosbeta
    vecoppouttoPerp = vn*cosbeta - trisideperp*sinbeta
    vecoppouttoPerpD0 = P3.Dot(vecoppouttoPerp, gb.pt)
    vecoppouttoPerpSide = P3.Dot(vecoppouttoPerp, tnodeopposite.p)

    bForeTriSide = (vecoppouttoPerpSide <= vecoppouttoPerpD0)
    barcrossing = barforerightBL if fromGoRight == bForeTriSide else barforerightBL.GetForeRightBL(barforerightBL.nodefore == tnodeopposite)
    assert barcrossing.GetNodeFore(barcrossing.nodeback == tnodeopposite) == gb.bar.GetNodeFore(bForeTriSide)

    vecoppouttoPerpDI = P3.Dot(vecoppouttoPerp, gb.bar.GetNodeFore(bForeTriSide).p)
    lamfromside = (vecoppouttoPerpD0 - vecoppouttoPerpSide) / (vecoppouttoPerpDI - vecoppouttoPerpSide)
    assert 0.0 <= lamfromside <= 1.0, lamfromside
    lambarcrossing = lamfromside if barcrossing.nodeback == tnodeopposite else 1.0 - lamfromside
    
    Dptbarcrossing = Along(lambarcrossing, barcrossing.nodeback.p, barcrossing.nodefore.p)
    Dsinbeta = P3.Dot(P3.ZNorm(Dptbarcrossing - gb.pt), vn)
    TOL_ZERO(Dsinbeta - sinbeta)
    lambarcrossingGoRight = (barcrossing.nodeback == tnodeopposite) == (bForeTriSide == fromGoRight)

    return GBarC(barcrossing, lambarcrossing, lambarcrossingGoRight)


def drivesetBFstartfromangle(drivebars, dpts, dptcls, ds, dsangle):
    dsseg, dslam = seglampos(ds, dptcls)
    gbStart = GBarT(drivebars, dsseg, dslam)
    gb, gbbackbar = gbStart.drivepointstartfromangle(dsangle, getbackbar=True)
    gbStart.gbBackbarC = gbbackbar
    gbStart.gbForebarC = gb
    return gbStart


def drivegeodesicRI(gbStart, drivebars, tridrivebarsmap, LRdirection=1, sideslipturningfactor=0.0, MAX_SEGMENTS=2000):
    gbs = [ gbStart, gbStart.gbForebarC ]
    Nconcavefolds = 0
    while True:
        gbFore = GBCrossBarRS(gbs[-1], gbs[-2].pt, sideslipturningfactor)
        if not gbFore or len(gbs) > MAX_SEGMENTS:
            print("exceeded MAX_SEGMENTS or off edge", MAX_SEGMENTS)
            gbs.append(None)
            break
        gbEnd = drivecurveintersectionfinder(drivebars, tridrivebarsmap, gbs[-1], gbFore, LRdirection=LRdirection)
        if gbEnd:
            gbs.append(gbEnd)
            break
        gbs.append(gbFore)
    return gbs


def okaypressed():
    print("Okay Pressed") 
    sketchplane = freecadutils.findobjectbylabel(qsketchplane.text())
    meshobject = freecadutils.findobjectbylabel(qmeshobject.text())
    alongwire = float(qalongwire.text())
    dsangle = float(qanglefilament.text())
    if len(qalongwireadvanceI.text()) != 0:
        alongwireadvanceI = float(qalongwireadvanceI.text())
        alongwireI = (alongwire + alongwireadvanceI) % 1.0
    else:
        alongwireI = None
    sideslipturningfactorZ = float(qsideslip.text())

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
    print("girth comparison", mandrelgirth, dptcls[-1])

    ds = Along(alongwire, dptcls[0], dptcls[-1])
    gbStart = drivesetBFstartfromangle(drivebars, dpts, dptcls, ds, dsangle)
    gbs = drivegeodesicRI(gbStart, drivebars, tridrivebarsmap, sideslipturningfactor=sideslipturningfactorZ)
    if gbs[-1] == None:
        Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbs[:-1]]), qoutputfilament.text())
        print("Collided with open edge")
        return
        
    dslanded = Along(gbs[-1].dclam, dptcls[gbs[-1].dcseg], dptcls[gbs[-1].dcseg + 1])
    alongwirelanded = InvAlong(dslanded, dptcls[0], dptcls[-1])
    if alongwireI is None:
        Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbs[:-1]]), qoutputfilament.text())
        print("alongwirelanded", alongwirelanded)
        qalongwireadvanceI.setText("%.3f" % ((alongwirelanded - alongwire + 1)%1))
        return
    
    gbsS = [ gbs[0].gbBackbarC ] + gbs[1:-1] + [ gbs[-1].gbForebarC ]
    drivebarsB = [ (gb.bar, gb.lam)  for gb in gbsS ]
    tridrivebarsmapB = dict((facetbetweenbars(drivebarsB[dseg][0], drivebarsB[dseg+1][0]).i, dseg)  for dseg in range(len(drivebarsB)-1))

    alongwireI1 = min([alongwireI, alongwireI+1], key=lambda X: abs(X - alongwirelanded))
    LRdirectionI = 1 if (alongwireI1 > alongwirelanded) else -1
    sideslipturningfactor = Maxsideslipturningfactor*LRdirectionI
    
    dsI = Along(alongwireI, dptcls[0], dptcls[-1])
    gbStartI = drivesetBFstartfromangle(drivebars, dpts, dptcls, dsI, dsangle+180.0)
    gbsI = drivegeodesicRI(gbStartI, drivebarsB, tridrivebarsmapB, sideslipturningfactor=sideslipturningfactor, LRdirection=LRdirectionI, MAX_SEGMENTS=len(gbs))

    if gbsI[-1] == None:
        print("Reversed path did not intersect")
        Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbs]), qoutputfilament.text())
        Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbsI[:-1]]), qoutputfilament.text())
        return

    for j in range(2):
        sideslipturningfactor *= 0.75
        gbStartIN = drivesetBFstartfromangle(drivebars, dpts, dptcls, dsI, dsangle+180.0)
        gbsIN = drivegeodesicRI(gbStartIN, drivebarsB, tridrivebarsmapB, sideslipturningfactor=sideslipturningfactor, LRdirection=LRdirectionI, MAX_SEGMENTS=len(gbs))
        if gbsIN[-1] != None:
            gbsI = gbsIN
            print("smaller sideslipturningfactor", sideslipturningfactor, "worked")
        else:
            break
            
    print("making join bit")
    dseg = tridrivebarsmapB[gbsI[-1].tbar.i]
    gbarT1 = gbsI[0]
    gbarT1.gbBackbarC, gbarT1.gbForebarC = gbarT1.gbForebarC, gbarT1.gbBackbarC
    gbsjoined = gbs[:dseg+1] + [ GBarC(gb.bar, gb.lam, not gb.bGoRight)  for gb in gbsI[-2:0:-1] ] + [ gbarT1 ]
    
    #Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbs[:dseg]]), qoutputfilament.text())
    #Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbsI]), qoutputfilament.text())
    Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbsjoined]), qoutputfilament.text())
    


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

Maxsideslipturningfactor = 0.26
mandrelradius = 110
mandrelgirth = 2*math.pi*mandrelradius
mandrelwindings = 10
mandrelrotations = 7
filamentoverlapadvance = 20/mandrelgirth
anglefilament = 30
mandreladvanceperwind = (filamentoverlapadvance/math.sin(math.radians(anglefilament)) + mandrelrotations)/mandrelwindings
TOL_ZERO(mandrelwindings*mandreladvanceperwind - mandrelrotations - filamentoverlapadvance/math.sin(math.radians(anglefilament)))

qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 500, 570, 350)
qw.setWindowTitle('Drive geodesic')
qsketchplane = freecadutils.qrow(qw, "Sketchplane: ", 15+35*0)
qmeshobject = freecadutils.qrow(qw, "Meshobject: ", 15+35*1 )

qalongwire = freecadutils.qrow(qw, "Along wire: ", 15+35*2, "0.51")
qanglefilament = freecadutils.qrow(qw, "Angle filament: ", 15+35*3, "%.1f" % anglefilament)

qalongwireadvanceI = freecadutils.qrow(qw, "Along wire adv.: ", 15+35*2, "%.2f" % mandreladvanceperwind, 260)
vlab = QtGui.QLabel("clear above to go one direction", qw)
vlab.move(20+260, 15+35*3+5)
qsideslip = freecadutils.qrow(qw, "Side slip: ", 15+35*4, "0", 260)

qoutputfilament = freecadutils.qrow(qw, "Output name: ", 15+35*4, "w1")
okButton = QtGui.QPushButton("Drive", qw)
okButton.move(180, 15+35*7)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  

qsketchplane.setText(freecadutils.getlabelofselectedsketch())
qmeshobject.setText(freecadutils.getlabelofselectedmesh())

qw.show()


