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


def GeoCrossAxisER(a, Vae, Vab, Isq, Isgn):
    # Solve: Isq*x.Lensq() - Square(P3.Dot(x, Vab)) = 0   for x = a + Vae*q
    # 0 = Isq*(a^2 + 2q a.Vae + q^2 Vae^2) - (a.Vab + Vae.Vab q)^2
    #   = Isq*(a^2 + 2q adf + q^2 Vae^2) - (adv + fdv q)^2

    fdv = P3.Dot(Vae, Vab)
    adv = P3.Dot(a, Vab)
    adf = P3.Dot(a, Vae)
    qA = Square(fdv) - Vae.Lensq()*Isq
    qB2 = adv*fdv - adf*Isq
    qC = Square(adv) - a.Lensq()*Isq
    if abs(qA) <= abs(qB2)*1e-7:
        if qB2 == 0:
            return -1.0
        q = -qC/(2*qB2)
    else:
        qdq = Square(qB2) - qA*qC
        if qdq < 0.0:
            #print("qdq", qdq)
            return -1.0
        qs = math.sqrt(qdq) / qA
        qm = -qB2 / qA
        q = qm + qs*Isgn
    # q = qs +- qm,  x = a + Vae*q,  Dot(x, Vab) same sign as Dot(Vcd, Vab)
    if abs(q) < 100:
        TOL_ZERO(qA*Square(q) + qB2*2*q + qC)
    return q


def GeoCrossAxisR(Ga, Gb, Gcfrom, lam, Geopposite, VabInterpolated):
    Gd = Along(lam, Ga, Gb)
    Vcd = Gd - Gcfrom
    if Vcd.Len() == 0:
        bEnd = True
        bAEcrossing = False
        q = 0
        Gx = Gcfrom
    else:
        bEnd = False
        cdDab = P3.Dot(Vcd, VabInterpolated)
        Isq = Square(cdDab) / Vcd.Lensq()
        Isgn = -1 if cdDab < 0 else 1
        qVae = GeoCrossAxisE(Ga - Gd, Geopposite - Ga, VabInterpolated, Isq, Isgn)
        qVbe = GeoCrossAxisE(Gb - Gd, Geopposite - Gb, -VabInterpolated, Isq, -Isgn)
        bAEcrossing = (abs(qVae - 0.5) < abs(qVbe - 0.5))
        q = qVae if bAEcrossing else qVbe
        Gx = (Ga + (Geopposite - Ga)*q) if bAEcrossing else (Gb + (Geopposite - Gb)*q)
        Dx = Gx - Gd
        TOL_ZERO(Isq - Square(P3.Dot(Dx, VabInterpolated)/Dx.Len()))
        TOL_ZERO(P3.Dot(Vcd, VabInterpolated)/Vcd.Len() - P3.Dot(Dx, VabInterpolated)/Dx.Len())
    return bAEcrossing, q, Gx, bEnd


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

def GBCrossBarR(gb, ptpushfrom, friccoeff):
    barforerightBL = gb.bar.GetForeRightBL(gb.bGoRight)
    if barforerightBL is None:
        return None
    tnodeopposite = barforerightBL.GetNodeFore(barforerightBL.nodeback == gb.bar.GetNodeFore(gb.bGoRight))
    vn = P3.ZNorm(gb.bar.nodefore.p - gb.bar.nodeback.p)
    pullforceFrom = P3.ZNorm(gb.pt - ptpushfrom)
    pullforcetnodeopposite = P3.ZNorm(gb.pt - tnodeopposite.p)
    fcnodeopposite = calcfriccoeff(pullforceFrom + pullforcetnodeopposite, vn)
    bForeTriSide = (friccoeff <= fcnodeopposite)
    if gb.bGoRight == bForeTriSide:
        barcrossing = barforerightBL
    else:
        barcrossing = barforerightBL.GetForeRightBL(barforerightBL.nodefore == tnodeopposite)
    assert barcrossing.GetNodeFore(barcrossing.nodeback == tnodeopposite) == gb.bar.GetNodeFore(bForeTriSide)
    
    fcnodeback, fcnodefore = calcfriccoeffbarEnds(pullforceFrom, vn)
    TOL_ZERO(fcnodeback - calcfriccoeff(pullforceFrom + vn, vn))
    TOL_ZERO(fcnodefore - calcfriccoeff(pullforceFrom - vn, vn))
    print(gb.bGoRight, fcnodeback, fcnodeopposite, fcnodefore)
#    bAEcrossing, q, Gx, bEnd = GeoCrossAxis(Na.p, Nb.p, c, lam, Ne.p, VabInterpolated)
    return
    
    if bGoRight:
        if bAEcrossing:
            bar = bar.barforeright.GetForeRightBL(bar.barforeright.nodeback == Nb)
            lam = q if bar.nodeback == Na else 1-q
            bGoRight = (bar.nodeback == Na)   
        else:
            bar = bar.barforeright
            lam = q if bar.nodeback == Nb else 1-q
            bGoRight = not (bar.nodeback == Nb)
    else:
        if bAEcrossing:
            bar = bar.barbackleft
            lam = q if bar.nodeback == Na else 1-q
            bGoRight = not (bar.nodeback == Na)
        else:
            bar = bar.barbackleft.GetForeRightBL(bar.barbackleft.nodeback == Na)
            lam = q if bar.nodeback == Nb else 1-q
            bGoRight = (bar.nodeback == Nb)
            
    c = bar.nodeback.p + (bar.nodefore.p - bar.nodeback.p)*lam
    TOL_ZERO((c - Gx).Len())
    return (d, bar, lam, bGoRight)


def GBCrossBarRS(gb, ptpushfrom, sideslipturningfactor):
    v = gb.bar.nodefore.p - gb.bar.nodeback.p
    vn = P3.ZNorm(v)
    pullforceFrom = P3.ZNorm(gb.pt - ptpushfrom)
    sinalpha = P3.Dot(vn, pullforceFrom)

    barforerightBL = gb.bar.GetForeRightBL(gb.bGoRight)
    if barforerightBL is None:
        return None
    tnodeopposite = barforerightBL.GetNodeFore(barforerightBL.nodeback == gb.bar.GetNodeFore(gb.bGoRight))

    trisidenorm = P3.ZNorm(P3.Cross(tnodeopposite.p - gb.bar.nodeback.p, v))
    trisideperp = P3.Cross(vn, trisidenorm)
    assert P3.Dot(trisideperp, tnodeopposite.p - gb.pt) >= 0.0
    
    sinbeta = sinalpha
    cosbeta = math.sqrt(max(0.0, 1.0 - sinbeta**2))

    vecoppoutto = vn*sinbeta + trisideperp*cosbeta
    vecoppouttoPerp = vn*cosbeta - trisideperp*sinbeta
    vecoppouttoPerpD0 = P3.Dot(vecoppouttoPerp, gb.pt)
    vecoppouttoPerpSide = P3.Dot(vecoppouttoPerp, tnodeopposite.p)

    bForeTriSide = (vecoppouttoPerpSide <= vecoppouttoPerpD0)
    barcrossing = barforerightBL if gb.bGoRight == bForeTriSide else barforerightBL.GetForeRightBL(barforerightBL.nodefore == tnodeopposite)
    assert barcrossing.GetNodeFore(barcrossing.nodeback == tnodeopposite) == gb.bar.GetNodeFore(bForeTriSide)

    vecoppouttoPerpDI = P3.Dot(vecoppouttoPerp, gb.bar.GetNodeFore(bForeTriSide).p)
    lamfromside = (vecoppouttoPerpD0 - vecoppouttoPerpSide) / (vecoppouttoPerpDI - vecoppouttoPerpSide)
    assert 0.0 <= lamfromside <= 1.0, lamfromside
    lambarcrossing = lamfromside if barcrossing.nodeback == tnodeopposite else 1.0 - lamfromside
    
    Dptbarcrossing = Along(lambarcrossing, barcrossing.nodeback.p, barcrossing.nodefore.p)
    Dsinbeta = P3.Dot(P3.ZNorm(Dptbarcrossing - gb.pt), vn)
    TOL_ZERO(Dsinbeta - sinbeta)
    lambarcrossingGoRight = (barcrossing.nodeback == tnodeopposite) == (bForeTriSide == gb.bGoRight)

    return GBarC(barcrossing, lambarcrossing, lambarcrossingGoRight)


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
        gbEnd = drivecurveintersectionfinder(drivebars, tridrivebarsmap, gbs[-1], gb, LRdirection=1)
        if gbEnd:
            gbs.append(gbEnd)
            break
        gbs.append(gb)

    angcross = gbEnd.drivecurveanglefromvec(gbs[-1].pt - gbs[-2].pt)
    dcross = Along(gbEnd.dclam, dptcls[gbEnd.dcseg], dptcls[gbEnd.dcseg+1])
    return gbs, dcross, angcross

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

    for i in range(20, 50, 5):
        break
        #GBCrossBarR(gbs[i], gbs[i-1].pt, friccoeff)
        h = GBCrossBarRS(gbs[i], gbs[i-1].pt, friccoeff)
        print(h.bar.i, gbs[i+1].bar.i, h.lam, gbs[i+1].lam, h.bGoRight, gbs[i+1].bGoRight)
        
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
qalongwireI = freecadutils.qrow(qw, "Along wire in: ", 15+35*2, "0.208", 260) # approx 0.214
qanglefilamentI = freecadutils.qrow(qw, "Angle fil. in: ", 15+35*3, "30.0", 260)

qoutputfilament = freecadutils.qrow(qw, "Output name: ", 15+35*4, "w1")
okButton = QtGui.QPushButton("Drive", qw)
okButton.move(180, 15+35*7)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  

qsketchplane.setText(freecadutils.getlabelofselectedsketch())
qmeshobject.setText(freecadutils.getlabelofselectedmesh())

qw.show()


