# -*- coding: utf-8 -*-
# Embed a curve into a mesh so we can head off in different directions and tell when it is crossed

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 

import os, sys, math, time
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])

from barmesh.basicgeo import I1, Partition1, P3, P2, Along
from curvesutils import isdiscretizableobject, discretizeobject
from trianglemeshutils import UsefulBoxedTriangleMesh, facetbetweenbars
from wireembeddingutils import TriangleCrossCutPlane, planecutembeddedcurve

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

# candidates for curveutils
def cumlengthlist(pts):
    ptcls = [ 0.0 ]
    for i in range(len(pts)):
        ptcls.append(ptcls[-1] + (pts[i] - pts[i-1]).Len())
    return ptcls

def seglampos(d, ptcls):
    i0, i1 = 0, len(ptcls)-1
    while i1 > i0 + 1:
        im = (i0 + i1)//2
        assert i0 < im < i1
        if ptcls[im] < d:
            i0 = im
        else:
            i1 = im
        assert ptcls[i0] <= d <= ptcls[i1]
    lam = (d - ptcls[i0]) / (ptcls[i1] - ptcls[i0])
    return i0, lam


def facetnormal(tbar):
    node2 = tbar.barforeright.GetNodeFore(tbar.barforeright.nodeback == tbar.nodefore)
    v2fore = tbar.nodefore.p - node2.p
    v2back = tbar.nodeback.p - node2.p
    return P3.ZNorm(P3.Cross(v2fore, v2back))

# main code here
utbm = UsefulBoxedTriangleMesh(meshobject)
startbar, startlam = utbm.FindClosestEdge(drivept0)
drivebars = planecutembeddedcurve(startbar, startlam, driveperpvec)

N = 5
startangtoline = 60
pts = [ Along(lam, bar.nodeback.p, bar.nodefore.p)  for bar, lam in drivebars ]
ptcls = cumlengthlist(pts)
for i in range(N):
    d = Along((i + 0.5)/N, ptcls[0], ptcls[-1])
    dseg, dlam = seglampos(d, ptcls)
    gpts = [ Along(dlam, pts[dseg], pts[dseg+1]) ]
    tbar = facetbetweenbars(drivebars[dseg][0], drivebars[dseg+1][0])
    tnorm = facetnormal(tbar)
    gpts.append(gpts[-1] + tnorm*10)
    Part.show(Part.makePolygon([Vector(*p)  for p in gpts]))



