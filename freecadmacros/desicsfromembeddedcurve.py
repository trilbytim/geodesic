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

def barfacetnormal(bar, bGoRight):
    nodeAhead = bar.GetNodeFore(bGoRight)
    nodeBehind = bar.GetNodeFore(not bGoRight)
    barAhead = bar.GetForeRightBL(bGoRight)
    barAheadGoRight = (barAhead.nodeback == nodeAhead)
    nodeOpposite = barAhead.GetNodeFore(barAheadGoRight)
    v2ahead = nodeAhead.p - nodeOpposite.p
    v2behind = nodeBehind.p - nodeOpposite.p
    return P3.ZNorm(P3.Cross(v2ahead, v2behind))

def facetnormal(tbar):
    #return barfacetnormal(tbar, True)

    node2 = tbar.barforeright.GetNodeFore(tbar.barforeright.nodeback == tbar.nodefore)
    v2fore = tbar.nodefore.p - node2.p
    v2back = tbar.nodeback.p - node2.p
    return P3.ZNorm(P3.Cross(v2fore, v2back))

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
    
def drivecurveintersectionfinder(drivebars, tridrivebarsmap, barlamB0, barlamB1):
    tbar = facetbetweenbars(barlamB0[0], barlamB1[0])
    if tbar.i not in tridrivebarsmap:
        return -1, -1.0
    dseg = tridrivebarsmap[tbar.i]
    Dtbar = facetbetweenbars(drivebars[dseg][0], drivebars[dseg+1][0])
    assert tbar == Dtbar
    dlam = trilinecrossing(tbar, drivebars[dseg], drivebars[dseg+1], barlamB0, barlamB1)
    if dlam == -1.0:
        return -1, -1.0
    print(tbar, dseg, dlam)
    return dseg, dlam

def drivecurveanglefromvec(drivebars, dpts, dseg, vec):
    tbar = facetbetweenbars(drivebars[dseg][0], drivebars[dseg+1][0])
    print(dseg, tbar)
    vsegN = P3.ZNorm(dpts[dseg+1] - dpts[dseg])
    tnorm = facetnormal(tbar)
    assert abs(P3.Dot(tnorm, vsegN)) < 0.001
    assert abs(P3.Dot(tnorm, vec)) < 0.001
    tperp = P3.Cross(vsegN, tnorm)
    ang = math.degrees(math.atan2(-P3.Dot(tperp, vec), P3.Dot(vsegN, vec)))
    return ang if ang > 0.0 else 360 + ang


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
    def __init__(self, drivebars, dpts, dseg, dlam):
        self.tbar = facetbetweenbars(drivebars[dseg][0], drivebars[dseg+1][0])
        self.pt = Along(dlam, dpts[dseg], dpts[dseg+1])
        self.vsegN = P3.ZNorm(dpts[dseg+1] - dpts[dseg])
        self.tnorm = facetnormal(self.tbar)
        self.tperp = P3.Cross(self.vsegN, self.tnorm)

    def drivepointstartfromangle(self, dangle):
        perpvec = -self.vsegN*math.sin(math.radians(dangle)) - self.tperp*math.cos(math.radians(dangle))
        perpvecDot = P3.Dot(perpvec, self.pt)
        bar, lam, bGoRight = TriangleExitCrossCutPlaneRight(self.tbar, perpvec, perpvecDot)
        res = GBarC(bar, lam, bGoRight)
        TOL_ZERO((self.tnorm - res.tnorm_incoming).Len(), "oo")
        return res
        

def drivegeodesic(drivebars, dpts, dptcls, ds, dsangle):
    dsseg, dslam = seglampos(ds, dptcls)
    gbStart = GBarT(drivebars, dpts, dsseg, dslam)
    ptprev = gbStart.pt
    gb = gbStart.drivepointstartfromangle(dsangle)
    ptcurr = gb.pt
    gpts = [ ptprev, ptcurr ]
    bar, lam, bGoRight = gb.bar, gb.lam, gb.bGoRight
    while len(gpts) < 450:
        prevgb = gb
        prevfacetnormal = barfacetnormal(prevgb.bar, not prevgb.bGoRight)
        TOL_ZERO((prevgb.tnorm_incoming - prevfacetnormal).Len(), "mm")
        gb = gb.GBCrossBar(ptprev)
        facetnormal = barfacetnormal(gb.bar, not gb.bGoRight)
        if not gb.bar:
            break
        ptprev = ptcurr
        ptcurr = gb.pt
        veccurr = ptcurr - ptprev
        fndot = P3.Dot(prevfacetnormal, P3.ZNorm(veccurr))
        TOL_ZERO(P3.Dot(facetnormal, P3.ZNorm(veccurr)), "mmnn")
        if fndot < -0.1:
            print(fndot, ptprev)
        dcseg, dclam = drivecurveintersectionfinder(drivebars, tridrivebarsmap, (prevgb.bar, prevgb.lam), (gb.bar, gb.lam))
        if dcseg != -1:
            cpt = Along(dclam, dpts[dcseg], dpts[dcseg+1])
            gpts.append(cpt)
            angcross = drivecurveanglefromvec(drivebars, dpts, dcseg, ptcurr - ptprev)
            dcross = Along(dclam, dptcls[dcseg], dptcls[dcseg+1])
            print("angcross", angcross, dcross)
            return gpts, dcross, angcross
        gpts.append(ptcurr)
    return gpts, -1, -1

dpts = [ Along(lam, bar.nodeback.p, bar.nodefore.p)  for bar, lam in drivebars ]
dptcls = cumlengthlist(dpts)

for dsangle in range(26, 33, 1):
    break
    ds = Along(0.1, dptcls[0], dptcls[-1])
    gpts, ds, dsangle = drivegeodesic(drivebars, dpts, dptcls, ds, dsangle)
    if gpts:
        Part.show(Part.makePolygon([Vector(*p)  for p in gpts]))
    print(ds, dsangle)

ds = Along(0.15, dptcls[0], dptcls[-1])
dsangle = 30
gpts1, ds1, dsangle1 = drivegeodesic(drivebars, dpts, dptcls, ds, dsangle)
print("pos ds1", ds1, dsangle1)
gpts2 = [ gpts1[-1] ]
#gpts2, ds2, dsangle2 = drivegeodesic(drivebars, dpts, dptcls, ds1, dsangle1+10)
Part.show(Part.makePolygon([Vector(*p)  for p in gpts1+gpts2[1:]]))
#print("Cylinder position angle advance degrees", 360*(ds2 - ds)/dptcls[-1])
#print("Leaving angle", dsangle, "Continuing angle", dsangle2)

# Cylinder position angle advance degrees 199.12638313124705
# Leaving angle 30 Continuing angle 33.82565262364904
