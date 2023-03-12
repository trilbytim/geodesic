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

def TriangleCrossTowards(bar, lam, bGoRight, ebar, ept):
    nodeAhead = bar.GetNodeFore(bGoRight)
    nodeBehind = bar.GetNodeFore(not bGoRight)
    barAhead = bar.GetForeRightBL(bGoRight)
    barAheadGoRight = (barAhead.nodeback == nodeAhead)
    nodeOpposite = barAhead.GetNodeFore(barAheadGoRight)
    barBehind = barAhead.GetForeRightBL(barAheadGoRight)
    barBehindGoRight = (barBehind.nodeback == nodeOpposite)
    assert nodeBehind == barBehind.GetNodeFore(barBehindGoRight)
    print("nodeback", bar.nodeback.p)
    print("nodefore", bar.nodefore.p)
    print(" nodeOpp", nodeOpposite.p)
     
    bpt = Along(lam, bar.nodeback.p, bar.nodefore.p)
    vpt = ept - bpt
    vopt = nodeOpposite.p - bpt

    vbar = nodeAhead.p - nodeBehind.p
    vbarleng = vbar.Len()
    axA = vbar*(1/vbarleng)
    axP = P3.ZNorm(P3.Cross(vpt, axA))
    axV = P3.Cross(axA, axP)
    fvpt = P2(P3.Dot(axV, vpt), P3.Dot(axA, vpt))
    fvopt = P2(P3.Dot(axV, vopt), P3.Dot(axA, vopt))
    fvptPerp = P2.APerp(fvpt)
    dopt = P2.Dot(fvptPerp, fvopt)
    bAheadSeg = (dopt < 0.0)
    if (ebar == (barAhead if not bAheadSeg else barBehind)):
        print("aiming back to ept case", dopt)
        bAheadSeg = not bAheadSeg
    barCrossing = (barAhead if bAheadSeg else barBehind)
    nodeAB = (nodeAhead if bAheadSeg else nodeBehind)
    fvAB = P2(0.0, vbarleng*((1-lam) if bGoRight == bAheadSeg else lam)*(1 if bAheadSeg else -1))
    DvAB = nodeAB.p - bpt
    DfvAB = P2(P3.Dot(axV, DvAB), P3.Dot(axA, DvAB))
    assert (fvAB - DfvAB).Len() < 0.0001, (fvAB, DfvAB)
    dnptAB = P2.Dot(fvptPerp, fvAB)
    barCrossingLamO = -dopt/(dnptAB - dopt)
    assert barCrossingLamO >= 0.0
    barCrossingLam = barCrossingLamO if (barCrossing.nodeback == nodeOpposite) == bGoRight else 1-barCrossingLamO
    barCrossingGoRight = (barCrossing.nodeback == nodeOpposite) == bAheadSeg
    
    return barCrossing, barCrossingLam, barCrossingGoRight

def BetweenBarListE(sbarlam, ebarlam):
    ept = Along(ebarlam[1], ebarlam[0].nodeback.p, ebarlam[0].nodefore.p)
    tctleft = TriangleCrossTowards(sbarlam[0], sbarlam[1], False, ebarlam[0], ept)
    #tctright = TriangleCrossTowards(sbarlam[0], sbarlam[1], True, ebarlam[0], ept)
    Dtptleft = Along(tctleft[1], tctleft[0].nodeback.p, tctleft[0].nodefore.p)
    #Dtptright = Along(tctright[1], tctright[0].nodeback.p, tctright[0].nodefore.p)
    print("Start")
    print("  ", sbarlam[0].nodeback.p)
    print("  ", sbarlam[0].nodefore.p)
    print("  ", sbarlam[1])
    print("TCT")
    tct = tctleft
    print("  ", tct[0].nodeback.p)
    print("  ", tct[0].nodefore.p)
    print("  ", tct[1], tct[2], Dtptleft)
    
    #print("LR TCT ", tctleft[1], tctright[1])
    print(Dtptleft)
    #print(Dtptright)
    print(ept)
    bGoRight = False # (Dtptright - ept).Len() < (Dtptleft - ept).Len()
    print("bGoRight", bGoRight)
    bar, lam, bGoRight = tctleft # tctright if bGoRight else tctleft
    print("00", bar.nodeback.p, bar.nodefore.p, lam, bGoRight)
    res = [ ]
    while bar != ebarlam[0]:
        res.append((bar, lam))
        bar, lam, bGoRight = TriangleCrossTowards(bar, lam, bGoRight, ebarlam[0], ept)
        print("11", bar.nodeback.p, bar.nodefore.p, lam, bGoRight)
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
    drivebars.append(ebarlam)
    break
#drivebars.extend(BetweenBarListE(drivebars[-1], startbarlam))
#drivebars.append(startbarlam)

epts = [ ]
for bar, lam in drivebars:
    ept = Along(lam, bar.nodeback.p, bar.nodefore.p)
    epts.append(ept)
Part.show(Part.makePolygon(epts))

facets = [ [ Vector(*bar.nodeback.p), Vector(*bar.nodefore.p), Vector(*TriangleNodeOpposite(bar, True).p) ]  for bar, lam in drivebars ]
#mesh = doc.addObject("Mesh::Feature", "m1")
#mesh.Mesh = Mesh.Mesh(facets)


