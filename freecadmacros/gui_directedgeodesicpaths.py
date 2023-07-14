# -*- coding: utf-8 -*-

# directional geodesics from embedded curve controlling endpoint

# Embed a curve into a mesh so we can head off in different directions and tell when it is crossed

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 
from PySide import QtGui, QtCore

import os, sys, math, time
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])
from barmesh.basicgeo import P2

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
    gbStart.dcseg = dsseg
    gbStart.dclam = dslam
    return gbStart


def drivegeodesicRI(gbStart, drivebars, tridrivebarsmap, LRdirection=1, sideslipturningfactor=0.0, MAX_SEGMENTS=2000, maxlength=-1):
    gbs = [ gbStart, gbStart.gbForebarC ]
    dlength = (gbs[1].pt - gbs[0].pt).Len()
    Nconcavefolds = 0
    while True:
        gbFore = GBCrossBarRS(gbs[-1], gbs[-2].pt, sideslipturningfactor)
        if not gbFore:
            print("gone off edge")
            gbs.append(None)
            break
        dlength += (gbFore.pt - gbs[-1].pt).Len()
        if len(gbs) > MAX_SEGMENTS or (maxlength != -1 and dlength > maxlength):
            print("exceeded MAX_SEGMENTS or length")
            gbs.append(None)
            break
        gbEnd = drivecurveintersectionfinder(drivebars, tridrivebarsmap, gbs[-1], gbFore, LRdirection=LRdirection)
        if gbEnd:
            gbs.append(gbEnd)
            break
        gbs.append(gbFore)
    return gbs

def makebicolouredwire(gbs, name, colfront=(1.0,0.0,0.0), colback=(0.0,0.3,0.0), leadcolornodes=-1):
    wire = Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbs]), qoutputfilament.text())
    if leadcolornodes == -1:
        leadcolornodes = min(len(gbs)//2, 40)
    wire.ViewObject.LineColorArray= [colfront]*leadcolornodes + [colback]*(len(gbs) - leadcolornodes)
    print("supposed to colour", wire.ViewObject)
    return wire


class DriveCurve:
    def __init__(self, drivebars):
        self.drivebars = drivebars
        self.tridrivebarsmap = dict((facetbetweenbars(drivebars[dseg][0], drivebars[dseg+1][0]).i, dseg)  for dseg in range(len(drivebars)-1))
        self.dpts = [ Along(lam, bar.nodeback.p, bar.nodefore.p)  for bar, lam in drivebars ]
        self.dptcls = cumlengthlist(self.dpts)

    def startalongangle(self, alongwire, dsangle):
        ds = Along(alongwire, self.dptcls[0], self.dptcls[-1])
        gbStart = drivesetBFstartfromangle(self.drivebars, self.dpts, self.dptcls, ds, dsangle)
        Dds = Along(gbStart.dclam, self.dptcls[gbStart.dcseg], self.dptcls[gbStart.dcseg + 1])
        Dalongwire = InvAlong(Dds, self.dptcls[0], self.dptcls[-1])
        TOL_ZERO(alongwire - Dalongwire)
        Ddsangle = 180 + gbStart.drivecurveanglefromvec(gbStart.gbBackbarC.pt - gbStart.gbForebarC.pt)
        TOL_ZERO(((dsangle - Ddsangle + 180)%360) - 180)
        return gbStart

    def endalongposition(self, gbEnd):
        dslanded = Along(gbEnd.dclam, self.dptcls[gbEnd.dcseg], self.dptcls[gbEnd.dcseg + 1])
        alongwirelanded = InvAlong(dslanded, self.dptcls[0], self.dptcls[-1])
        angcross = gbEnd.drivecurveanglefromvec(gbEnd.gbForebarC.pt - gbEnd.gbBackbarC.pt)
        print("angcross", angcross)
        return alongwirelanded



def okaypressed():
    print("Okay Pressed ", qcombomode.currentIndex())
    combofoldbackmode = qcombomode.currentIndex()
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
    maxlength = float(qmaxlength.text())

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
    drivecurve = DriveCurve(drivebars)
    print("girth comparison", mandrelgirth, drivecurve.dptcls[-1])

    gbStart = drivecurve.startalongangle(alongwire, dsangle)

    fLRdirection = 1 if ((dsangle + 360)%360) < 180.0 else -1
    if combofoldbackmode != 0:
        fLRdirection = -fLRdirection
        
    print("doing alongwire %.2f foldback=%d  alongwireI %.2f" % (alongwire, fLRdirection, alongwireI or 0))
    gbs = drivegeodesicRI(gbStart, drivecurve.drivebars, drivecurve.tridrivebarsmap, LRdirection=fLRdirection, sideslipturningfactor=sideslipturningfactorZ, maxlength=maxlength)
    if gbs[-1] == None:
        Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbs[:-1]]), qoutputfilament.text())
        return
        
    alongwirelanded = drivecurve.endalongposition(gbs[-1])
    if alongwireI is None:
        wirelength = sum((gb2.pt - gb1.pt).Len()  for gb1, gb2 in zip(gbs, gbs[1:]))
        makebicolouredwire(gbs, qoutputfilament.text(), colfront=(1.0,0.0,0.0), 
                                                        colback =(0.0,0.6,0.0) if abs(dsangle) < 90 else (0.0,0.0,0.9))
        print("alongwirelanded %4f  leng=%.2f   ** setting AlngWre advance to the difference" % (alongwirelanded, wirelength))
        qalongwireadvanceI.setText("%.4f" % ((alongwirelanded - alongwire + 1)%1))
        return
    
    gbsS = [ gbs[0].gbBackbarC ] + gbs[1:-1] + [ gbs[-1].gbForebarC ]
    drivebarsB = [ (gb.bar, gb.lam)  for gb in gbsS ]
    tridrivebarsmapB = dict((facetbetweenbars(drivebarsB[dseg][0], drivebarsB[dseg+1][0]).i, dseg)  for dseg in range(len(drivebarsB)-1))

    alongwireI1 = min([alongwireI, alongwireI+1], key=lambda X: abs(X - alongwirelanded))
    LRdirectionI = 1 if (alongwireI1 > alongwirelanded) else -1
    sideslipturningfactor = Maxsideslipturningfactor*LRdirectionI
    
    dsangleI = dsangle+180.0 if combofoldbackmode == 0 else 180.0-dsangle
    print("dsangleI", dsangleI, dsangle, combofoldbackmode)
    gbStartI = drivecurve.startalongangle(alongwireI, dsangleI)
    gbsI = drivegeodesicRI(gbStartI, drivebarsB, tridrivebarsmapB, sideslipturningfactor=sideslipturningfactor, LRdirection=LRdirectionI, MAX_SEGMENTS=len(gbs))
    
    if gbsI[-1] == None:
        print("Reversed path did not intersect")
        Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbs]), qoutputfilament.text())
        Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbsI[:-1]]), qoutputfilament.text())
        return

    for j in range(2):
        sideslipturningfactor *= 0.75
        gbsIN = drivegeodesicRI(gbStartI, drivebarsB, tridrivebarsmapB, sideslipturningfactor=sideslipturningfactor, LRdirection=LRdirectionI, MAX_SEGMENTS=len(gbs))
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
    
    makebicolouredwire(gbsjoined, qoutputfilament.text(), colfront=(1.0,0.0,0.0) if fLRdirection == -1 else (0.0,0.0,1.0), 
                                                          colback=(0.7,0.7,0.0), leadcolornodes=dseg+1)
    

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

mandrelradius = 110  # fc6 file
mandrelradius = 125  # PV file


mandrelgirth = 2*math.pi*mandrelradius
mandrelwindings = 10
mandrelrotations = 7
filamentoverlapadvance = 20/mandrelgirth
anglefilament = -30
mandreladvanceperwind = (filamentoverlapadvance/math.sin(math.radians(anglefilament)) + mandrelrotations)/mandrelwindings
TOL_ZERO(mandrelwindings*mandreladvanceperwind - mandrelrotations - filamentoverlapadvance/math.sin(math.radians(anglefilament)))
maxlength = 6000

qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 500, 570, 350)
qw.setWindowTitle('Drive geodesic')
qsketchplane = freecadutils.qrow(qw, "Sketchplane: ", 15+35*0)
qmeshobject = freecadutils.qrow(qw, "Meshobject: ", 15+35*1 )

qalongwire = freecadutils.qrow(qw, "Along wire: ", 15+35*2, "0.51")
qanglefilament = freecadutils.qrow(qw, "Angle filament: ", 15+35*3, "%.1f" % anglefilament)

qmaxlength = freecadutils.qrow(qw, "maxlength: ", 15+35*1, "%.2f" % maxlength, 260)
qalongwireadvanceI = freecadutils.qrow(qw, "AlngWre adv(+!): ", 15+35*2, "", 260)

vlab = QtGui.QLabel("clear above to go one direction", qw)
vlab.move(20+260, 15+35*3+5)
qsideslip = freecadutils.qrow(qw, "Side slip: ", 15+35*4, "0", 260)
qcombomode = QtGui.QComboBox(qw)
qcombomode.move(120+260, 15+35*5)
qcombomode.addItem("Mode0 normal")
qcombomode.addItem("Mode1 foldback")
#qcombomode.addItem("Mode2 reflect")
qcombomode.setCurrentIndex(1)

qoutputfilament = freecadutils.qrow(qw, "Output name: ", 15+35*4, "w1")
okButton = QtGui.QPushButton("Drive", qw)
okButton.move(180, 15+35*7)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  

qsketchplane.setText(freecadutils.getlabelofselectedsketch())
qmeshobject.setText(freecadutils.getlabelofselectedmesh())

qw.show()

# When running on PV.Fcad
# ang=-30 pos=0.51  adv=0.57
# ang=-145 pos=0.08  adv=0.46 (actually -0.54)
# we have now advanced to 0.54, or 0.03* 2*pi*125 = 23.5mm


# Suppose we want to advance one tape width per switchback
# tapewidth = 10, tiltedtapewidth = 10/abs(sin(ang)) = 20
# advance = 20/girth = 0.0254
# equalize this advance between the two steps

# ang=-30 pos=0.51  adv=0.564 + 0.0254/2 = 0.5767
# return is 
#  ang=-(180-30) = -150
#  pos=0.51 + 0.5767 = 0.0867
#  adv=0.51 + 0.0254 - 0.0867 = 0.4487

#   this should be 0.017 instead of 0.0254 because of the tilt
# ang=-50 pos=0.51  adv=0.488 + 0.0254/2 = 0.5007
# return is 
#  ang=-(180-50) = -130
#  pos=0.51 + 0.5007 = 0.0107
#  adv=0.51 + 0.0254 - 0.0107 = 0.5247

