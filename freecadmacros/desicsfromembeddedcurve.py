# -*- coding: utf-8 -*-

# directional geodesics from embedded curve

# Embed a curve into a mesh so we can head off in different directions and tell when it is crossed

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 

import os, sys, math, time
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])

from barmesh.basicgeo import I1, Partition1, P3, P2, Along
from curvesutils import isdiscretizableobject, discretizeobject
from curvesutils import cumlengthlist, seglampos
from trianglemeshutils import UsefulBoxedTriangleMesh, facetbetweenbars
from wireembeddingutils import TriangleCrossCutPlane, planecutembeddedcurve
from wireembeddingutils import showdrivebarsmesh, showdrivebarscurve
from geodesicutils import GeoCrossAxis, GeoCrossBar, TOL_ZERO

doc = App.ActiveDocument
gui = App.Gui
sel = Gui.Selection.getSelection()

# drive curve definition (cut plane)
drivept0 = P3(110.160362, -1.82, 1.1216061)
driveperpvec = P3(0, 1, 0)

meshobject = None
for s in sel:
    if hasattr(s, "Mesh") and isinstance(s.Mesh, Mesh.Mesh):
        meshobject = s.Mesh
if not meshobject:
    print("Need a mesh object selected")

def barfacetnormal(bar, bGoRight, ptcommon=None):
    nodeAhead = bar.GetNodeFore(bGoRight)
    nodeBehind = bar.GetNodeFore(not bGoRight)
    barAhead = bar.GetForeRightBL(bGoRight)
    barAheadGoRight = (barAhead.nodeback == nodeAhead)
    nodeOpposite = barAhead.GetNodeFore(barAheadGoRight)
    ptc = nodeOpposite.p if ptcommon is None else ptcommon
    v2ahead = nodeAhead.p - ptc
    v2behind = nodeBehind.p - ptc
    return P3.ZNorm(P3.Cross(v2ahead, v2behind))

def TOL_ZERO(X, msg=""):
    if not (abs(X) < 0.0001):
        print("TOL_ZERO fail", X, msg)
        assert False

def InvAlong(v, a, b):
    return (v - a)/(b - a)

def TriangleExitCrossCutPlaneRight(tbar, perpvec, perpvecDot):
    node2 = tbar.barforeright.GetNodeFore(tbar.barforeright.nodeback == tbar.nodefore)
    dpvdnodefore = P3.Dot(perpvec, tbar.nodefore.p)
    dpvdnodeback = P3.Dot(perpvec, tbar.nodeback.p)
    dpvdnode2 = P3.Dot(perpvec, node2.p)
    assert tbar.nodeback.i < tbar.nodefore.i
    
    if dpvdnodefore <= perpvecDot <= dpvdnodeback:
        tbarlam = InvAlong(perpvecDot,dpvdnodeback, dpvdnodefore)
        return tbar, tbarlam, False

    if dpvdnodeback <= perpvecDot <= dpvdnode2:
        tbarlam = InvAlong(perpvecDot, dpvdnodeback, dpvdnode2)
        barbackright = tbar.barforeright.GetForeRightBL(tbar.barforeright.nodeback == tbar.nodefore)
        assert barbackright.nodeback.i < barbackright.nodefore.i
        return barbackright, tbarlam, True

    if dpvdnode2 <= perpvecDot <= dpvdnodefore:
        bforerightFore = (node2.i < tbar.nodefore.i)
        tbarlamF = InvAlong(perpvecDot, dpvdnode2, dpvdnodefore)
        tbarlam = tbarlamF if bforerightFore else 1.0 - tbarlamF
        return tbar.barforeright, tbarlam, bforerightFore

    return None, 0, False         


# main code here
utbm = UsefulBoxedTriangleMesh(meshobject)
startbar, startlam = utbm.FindClosestEdge(drivept0)
drivebars = planecutembeddedcurve(startbar, startlam, driveperpvec)
# showdrivebarsmesh(drivebars, doc, meshname="m1"):
# showdrivebarscurve(drivebars, doc)
tridrivebarsmap = dict((facetbetweenbars(drivebars[dseg][0], drivebars[dseg+1][0]).i, dseg)  for dseg in range(len(drivebars)-1))


def triclambarlam(tbar, barlam):
    bar, lam = barlam
    if bar == tbar:
        return lam
    if bar == tbar.barforeright:
        return 1.0 + (lam if tbar.barforeright.nodeback == tbar.nodefore else 1.0 - lam)
    assert bar == tbar.barforeright.GetForeRightBL(tbar.barforeright.nodeback == tbar.nodefore)
    return 2.0 + (1.0 - lam)

def trilamrightangleproj(clam):
    if clam <= 1.0:
        return P2(0.0, clam)
    if clam <= 2.0:
        return P2(clam - 1.0, 2.0 - clam)
    return P2(3.0 - clam, 0.0)
    
def trilinecrossing(tbar, barlamA0, barlamA1, barlamB0, barlamB1):
    clamA0 = triclambarlam(tbar, barlamA0)
    clamA1 = triclambarlam(tbar, barlamA1)
    clamB0 = triclambarlam(tbar, barlamB0)
    clamB1 = triclambarlam(tbar, barlamB1)
    cseq = [ (clamA0, 0), (clamA1, 0), (clamB0, 1), (clamB1, 1) ]
    cseq.sort()
    if cseq[0][1] == cseq[1][1] or cseq[1][1] == cseq[2][1]:
        return -1.0
    # project clam values into simple right angle triangle, which is a linear transform of the real triangle, so the parametrized intersection position remains the same
    tA0 = trilamrightangleproj(clamA0)
    tA1 = trilamrightangleproj(clamA1)
    tB0 = trilamrightangleproj(clamB0)
    tB1 = trilamrightangleproj(clamB1)
    vBperp = P2.CPerp(tB1 - tB0)
    vBperpdot = P2.Dot(vBperp, tB0)
    vBperpdotA0 = P2.Dot(vBperp, tA0)
    vBperpdotA1 = P2.Dot(vBperp, tA1)
    lamA = (vBperpdot - vBperpdotA0)/(vBperpdotA1 - vBperpdotA0)
    assert 0.0 <= lamA <= 1.0, lamA
    DptA0 = Along(barlamA0[1], barlamA0[0].nodeback.p, barlamA0[0].nodefore.p)
    DptA1 = Along(barlamA1[1], barlamA1[0].nodeback.p, barlamA1[0].nodefore.p)
    DptB0 = Along(barlamB0[1], barlamB0[0].nodeback.p, barlamB0[0].nodefore.p)
    DptB1 = Along(barlamB1[1], barlamB1[0].nodeback.p, barlamB1[0].nodefore.p)
    DptcrossA = Along(lamA, DptA0, DptA1)
    DvB = DptB1 - DptB0
    DlamB = P3.Dot(DptcrossA - DptB0, DvB)/DvB.Lensq()
    DptcrossB = Along(DlamB, DptB0, DptB1)
    assert (DptcrossB - DptcrossA).Len() < 0.001
    return lamA   
    
def drivecurveintersectionfinder(drivebars, tridrivebarsmap, gb0, gb1):
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
    res.dcseg = dseg
    res.dclam = dlam
    TOL_ZERO(P3.Cross(gb1.tnorm_incoming, res.tnorm).Len())
    res.tnorm_incoming = gb1.tnorm_incoming
    return res
    

class GBarC:
    def __init__(self, bar, lam, bGoRight):
        self.bar = bar
        self.lam = lam
        self.bGoRight = bGoRight
        self.pt = Along(self.lam, self.bar.nodeback.p, self.bar.nodefore.p)
        self.tnorm_incoming = barfacetnormal(self.bar, not self.bGoRight)

    def GBCrossBar(self, ptpushfrom):
        c, bar, lam, bGoRight = GeoCrossBar(ptpushfrom, self.bar, self.lam, self.bGoRight)
        res = GBarC(bar, lam, bGoRight)
        TOL_ZERO((c - self.pt).Len())
        return res

class GBarT:
    def __init__(self, drivebars, dseg, dlam):
        self.tbar = facetbetweenbars(drivebars[dseg][0], drivebars[dseg+1][0])
        dpt = Along(drivebars[dseg][1], drivebars[dseg][0].nodeback.p, drivebars[dseg][0].nodefore.p)
        dpt1 = Along(drivebars[dseg+1][1], drivebars[dseg+1][0].nodeback.p, drivebars[dseg+1][0].nodefore.p)
        self.pt = Along(dlam, dpt, dpt1)
        self.vsegN = P3.ZNorm(dpt1 - dpt)
        self.tnorm = barfacetnormal(self.tbar, True)
        self.tperp = P3.Cross(self.vsegN, self.tnorm)

    def drivepointstartfromangle(self, dangle):
        perpvec = -self.vsegN*math.sin(math.radians(dangle)) - self.tperp*math.cos(math.radians(dangle))
        perpvecDot = P3.Dot(perpvec, self.pt)
        bar, lam, bGoRight = TriangleExitCrossCutPlaneRight(self.tbar, perpvec, perpvecDot)
        res = GBarC(bar, lam, bGoRight)
        TOL_ZERO((self.tnorm - res.tnorm_incoming).Len(), "oo")
        return res

    def drivecurveanglefromvec(self, vec):
        ang = math.degrees(math.atan2(-P3.Dot(self.tperp, vec), P3.Dot(self.vsegN, vec)))
        return ang if ang > 0.0 else 360 + ang
        

def drivegeodesic(drivebars, dpts, dptcls, ds, dsangle):
    dsseg, dslam = seglampos(ds, dptcls)
    gbStart = GBarT(drivebars, dsseg, dslam)
    gb = gbStart.drivepointstartfromangle(dsangle)
    gbs = [ gbStart, gb ]
    gbEnd = None
    Nconcavefolds = 0
    while gbEnd is None:
        gb = gbs[-1].GBCrossBar(gbs[-2].pt)
        if not gb.bar or len(gbs) > 450:
            return gbs, -1, -1
        gbEnd = drivecurveintersectionfinder(drivebars, tridrivebarsmap, gbs[-1], gb)
        gbs.append(gb if gbEnd is None else gbEnd)
        
        while len(gbs) >= 3:
            veccurr = gbs[-1].pt - gbs[-2].pt
            TOL_ZERO(P3.Dot(gbs[-1].tnorm_incoming, P3.ZNorm(veccurr)))
            fndot = P3.Dot(P3.ZNorm(veccurr), gbs[-2].tnorm_incoming)
            if fndot >= 0.0:
                break
            Nconcavefolds += 1
            del gbs[-2]
            Dprevtnorm_incoming = gbs[-1].tnorm_incoming
            gbs[-1].tnorm_incoming = barfacetnormal(gbs[-1].bar, not gbs[-1].bGoRight, gbs[-2].pt)
            assert P3.Dot(gbs[-1].tnorm_incoming, Dprevtnorm_incoming) > 0.9

    print("Nconcavefolds removed", Nconcavefolds, "leaving", len(gbs))
    angcross = gbEnd.drivecurveanglefromvec(gbs[-2].pt - gbs[-1].pt)
    dcross = Along(gbEnd.dclam, dptcls[gbEnd.dcseg], dptcls[gbEnd.dcseg+1])
    print("angcross", angcross, dcross)
    return gbs, dcross, angcross


dpts = [ Along(lam, bar.nodeback.p, bar.nodefore.p)  for bar, lam in drivebars ]
dptcls = cumlengthlist(dpts)

for dsangle in range(26, 33, 1):
    break
    ds = Along(0.1, dptcls[0], dptcls[-1])
    gbs, ds, dsangle = drivegeodesic(drivebars, dpts, dptcls, ds, dsangle)
    if gbs:
        Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gps]))
    print(ds, dsangle)

ds = Along(0.15, dptcls[0], dptcls[-1])
dsangle = 30
gbs1, ds1, dsangle1 = drivegeodesic(drivebars, dpts, dptcls, ds, dsangle)
print("pos ds1", ds1, dsangle1)
gbs2 = [ gbs1[-1] ]
#gbs2, ds2, dsangle2 = drivegeodesic(drivebars, dpts, dptcls, ds1, dsangle1+30)
Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbs1+gbs2[1:]]))
#print("Cylinder position angle advance degrees", 360*(ds2 - ds)/dptcls[-1])
#print("Leaving angle", dsangle, "Continuing angle", dsangle2)

# Cylinder position angle advance degrees 199.12638313124705
# Leaving angle 30 Continuing angle 33.82565262364904
