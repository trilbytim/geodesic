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
        barbackleft = tbar.barforeright.GetForeRightBL(tbar.barforeright.nodeback == tbar.nodefore)
        assert barbackleft.nodeback.i < barbackleft.nodefore.i
        return barbackleft, tbarlam, True

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

tridrivebarsmap = dict((drivebars[dseg][0].i, dseg)  for dseg in range(len(drivebars)))

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




N = 5
startangtoline = 30
vang = P2(math.cos(math.radians(startangtoline)), math.sin(math.radians(startangtoline)))
print("vang", vang)
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
        tbar = facetbetweenbars(prevbar, bar)
        print(tbar, prevbar.GetForeRightBL(prevbGoRight), bar.GetForeRightBL(not bGoRight))
        if tbar.i in tridrivebarsmap:
            break
        gpts.append(ptcurr)
    Part.show(Part.makePolygon([Vector(*p)  for p in gpts]))
    break

