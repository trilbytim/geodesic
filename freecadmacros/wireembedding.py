# -*- coding: utf-8 -*-
# Embed a curve into a mesh so we can head off in different directions and tell when it is crossed

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 

import os, sys, math, time
sys.path.append(os.path.join(os.path.split(__file__)[0]))

from barmesh.tribarmes import TriangleBarMesh, TriangleBar, MakeTriangleBoxing
from barmesh.basicgeo import I1, Partition1, P3, P2, Along
from FreeCAD import Vector
from curvesutils import isdiscretizableobject, discretizeobject

doc = App.ActiveDocument
gui = App.Gui
sel = Gui.Selection.getSelection()

def clamp(num, min_value, max_value):
    return max(min(num, max_value), min_value)

class UsefulBoxedTriangleMesh:
    def __init__(self, meshobject):
        global tbarmesh, tboxing, hitreg, nhitreg
        trpts = [ sum(f.Points, ())  for f in meshobject.Facets ]
        print("building tbarmesh ", len(trpts))
        self.tbarmesh = TriangleBarMesh()
        self.tbarmesh.BuildTriangleBarmesh(trpts)
        assert len(self.tbarmesh.nodes) == meshobject.CountPoints
        assert len(self.tbarmesh.bars) == meshobject.CountEdges
        self.tboxing = MakeTriangleBoxing(self.tbarmesh)
        self.hitreg = [0]*len(self.tbarmesh.bars)
        self.nhitreg = 0
        
    def FindClosestEdge(self, p, r=3):
        self.nhitreg += 1
        barclosest = None
        barclosestlam = 0.0
        for ix, iy in self.tboxing.CloseBoxeGenerator(p.x, p.x, p.y, p.y, r):
            tbox = self.tboxing.boxes[ix][iy]
            for i in tbox.edgeis:
                if self.hitreg[i] != self.nhitreg:
                    bar = self.tbarmesh.bars[i]
                    p0, p1 = bar.nodeback.p, bar.nodefore.p
                    lv = p - p0
                    v = p1 - p0
                    vsq = v.Lensq()
                    lam = P3.Dot(v, lv) / vsq
                    lam = clamp(lam, 0.0, 1.0)
                    vd = lv - v * lam
                    assert lam == 0.0 or lam == 1.0 or abs(P3.Dot(vd, v)) < 0.0001
                    vdlen = vd.Len()
                    if vdlen < r:
                        r = vdlen
                        barclosest = bar
                        barclosestlam = lam
                    self.hitreg[i] = self.nhitreg
        return barclosest, barclosestlam

    
meshobject = None
drivepts = None
for s in sel:
    if hasattr(s, "Mesh") and isinstance(s.Mesh, Mesh.Mesh):
        meshobject = s.Mesh
    elif isdiscretizableobject(s):
        drivepts = discretizeobject(s, deflection=0.2)
if not meshobject:
    print("Need a mesh object selected")
if not drivepts:
    print("Need a linearizable object selected")

print(drivepts[0], drivepts[-1])
utbm = UsefulBoxedTriangleMesh(meshobject)
print(utbm)

epts = [ ]
for pt in drivepts:
    bar, lam = utbm.FindClosestEdge(P3(*pt))
    ept = Along(lam, bar.nodeback.p, bar.nodefore.p)
    epts.append(ept)
Part.show(Part.makePolygon(epts))



