# -*- coding: utf-8 -*-
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
#from wireembeddingutils import showdrivebarsmesh, showdrivebarscurve
from geodesicutils import GeoCrossAxis, GeoCrossBar

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

def facetnormal(tbar):
    node2 = tbar.barforeright.GetNodeFore(tbar.barforeright.nodeback == tbar.nodefore)
    v2fore = tbar.nodefore.p - node2.p
    v2back = tbar.nodeback.p - node2.p
    return P3.ZNorm(P3.Cross(v2fore, v2back))

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

def drivepointstartfromangle(drivebars, dseg, dlam, dangle):
    tbar = facetbetweenbars(drivebars[dseg][0], drivebars[dseg+1][0])
    pt = Along(dlam, pts[dseg], pts[dseg+1])
    vsegN = P3.ZNorm(pts[dseg+1] - pts[dseg])
    tnorm = facetnormal(tbar)
    tperp = P3.Cross(vsegN, tnorm)
    perpvec = -vsegN*math.sin(math.radians(dangle)) - tperp*math.cos(math.radians(dangle))
    perpvecDot = P3.Dot(perpvec, pt)
    bar, lam, bGoRight = TriangleExitCrossCutPlaneRight(tbar, perpvec, perpvecDot)
    return pt, bar, lam, bGoRight

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
    return dseg, dlam

N = 5
startangtoline = 30
pts = [ Along(lam, bar.nodeback.p, bar.nodefore.p)  for bar, lam in drivebars ]
ptcls = cumlengthlist(pts)
for i in range(N):
    d = Along((i + 0.5)/N, ptcls[0], ptcls[-1])
    dseg, dlam = seglampos(d, ptcls)
    ptprev, bar, lam, bGoRight = drivepointstartfromangle(drivebars, dseg, dlam, startangtoline)
    if bar == None:
        continue
    ptcurr = Along(lam, bar.nodeback.p, bar.nodefore.p)
    gpts = [ ptprev, ptcurr ]
    while len(gpts) < 350:
        prevbar, prevlam, prevbGoRight = bar, lam, bGoRight
        Dc, bar, lam, bGoRight = GeoCrossBar(ptprev, bar, lam, bGoRight)
        if not bar:
            print("jjjk ", c, bar, lam, bGoRight)
            break
        ptprev = ptcurr
        ptcurr = Along(lam, bar.nodeback.p, bar.nodefore.p)
        assert (ptprev - Dc).Len() < 0.001
        dcseg, dclam = drivecurveintersectionfinder(drivebars, tridrivebarsmap, (prevbar, prevlam), (bar, lam))
        if dcseg != -1:
            cpt = Along(dclam, pts[dcseg], pts[dcseg+1])
            gpts.append(cpt)
            break
        gpts.append(ptcurr)
    Part.show(Part.makePolygon([Vector(*p)  for p in gpts]))
    break

