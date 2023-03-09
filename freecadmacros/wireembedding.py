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

def TriangleNodeOpposite(bar, bGoRight):
    bartop = bar.GetForeRightBL(bGoRight)
    if bartop != None:
        return bartop.GetNodeFore(bartop.nodeback == bar.GetNodeFore(bGoRight))
    return None

def BarCrossTowards(bar, lam, ebar, elam):
    assert bar != ebar
    bpt = Along(lam, bar.nodeback.p, bar.nodefore.p)
    ept = Along(elam, ebar.nodeback.p, ebar.nodefore.p)
    axD = P3.ZNorm(ept - bpt)
    axP = P3.Cross(axD, bar.nodefore.p - bar.nodeback.p)
    axA = P3.Cross(axP, axD)
    print("RS ", P3.Dot(axA, bar.nodefore.p - bar.nodeback.p))
    return ebar, elam

    barforeright = bar.barforeright
    noderight = barforeright.GetNodeFore(barforeright.nodeback == bar.nodefore)
    barbackright = barforeright.GetForeRightBL(barforeright.nodeback == bar.nodefore)
    barbackleft = bar.barbackleft
    nodeleft = barbackleft.GetNodeFore(barbackleft.nodeback == bar.nodeback)
    barforeleft = barbackleft.GetForeRightBL(barbackleft.nodeback == bar.nodeback)
    if ebar in [ bar, barforeright, barbackright, barforeleft, barbackleft ]:
        return ebar, elam

    axB = P3.ZNorm(bar.nodefore.p - bar.nodeback.p)
    axP = P3.ZNorm(P3.Cross(noderight.p - nodeleft.p, axB))
    axR = P3.Cross(axB, axP)

    dvec = ept - bpt
    return ebar, elam

def BetweenBarListE(barlam, ebarlam):
    res = [ ]
    while barlam[0] != ebarlam[0]:
        barlam = BarCrossTowards(barlam[0], barlam[1], ebarlam[0], ebarlam[1])
        res.append(barlam)
        if len(res) > 10:
            print("failed to generate between bar list")
            break
    return res
    
    
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

startbarlam = utbm.FindClosestEdge(P3(*drivepts[0]))
drivebars = [ startbarlam ]
for pt in drivepts[1:-1]:
    ebarlam = utbm.FindClosestEdge(P3(*pt))
    drivebars.extend(BetweenBarListE(drivebars[-1], ebarlam))
drivebars.extend(BetweenBarListE(drivebars[-1], startbarlam))

epts = [ ]
for bar, lam in drivebars:
    ept = Along(lam, bar.nodeback.p, bar.nodefore.p)
    epts.append(ept)
#Part.show(Part.makePolygon(epts))

facets = [ [ Vector(*bar.nodeback.p), Vector(*bar.nodefore.p), Vector(*TriangleNodeOpposite(bar, True).p) ]  for bar, lam in drivebars ]
mesh = doc.addObject("Mesh::Feature", "m1")
mesh.Mesh = Mesh.Mesh(facets)


