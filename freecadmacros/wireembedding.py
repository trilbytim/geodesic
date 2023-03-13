# -*- coding: utf-8 -*-
# Embed a curve into a mesh so we can head off in different directions and tell when it is crossed

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 

# nix shell github:nixos/nixpkgs/nixos-unstable#godot_4

import os, sys, math, time
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])

from barmesh.tribarmes import TriangleBarMesh, TriangleBar, MakeTriangleBoxing
from barmesh.basicgeo import I1, Partition1, P3, P2, Along
from FreeCAD import Vector
from curvesutils import isdiscretizableobject, discretizeobject
from trianglemeshutils import UsefulBoxedTriangleMesh

doc = App.ActiveDocument
gui = App.Gui
sel = Gui.Selection.getSelection()

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
    barCrossingLam = barCrossingLamO if (barCrossing.nodeback == nodeOpposite) else 1-barCrossingLamO
    barCrossingGoRight = (barCrossing.nodeback == nodeOpposite) == bAheadSeg
    
    return barCrossing, barCrossingLam, barCrossingGoRight

def BetweenBarListE(sbarlam, ebarlam):
    ept = Along(ebarlam[1], ebarlam[0].nodeback.p, ebarlam[0].nodefore.p)
    tctleft = TriangleCrossTowards(sbarlam[0], sbarlam[1], False, ebarlam[0], ept)
    tctright = TriangleCrossTowards(sbarlam[0], sbarlam[1], True, ebarlam[0], ept)
    Dtptleft = Along(tctleft[1], tctleft[0].nodeback.p, tctleft[0].nodefore.p)
    Dtptright = Along(tctright[1], tctright[0].nodeback.p, tctright[0].nodefore.p)
    bGoRight = (Dtptright - ept).Len() < (Dtptleft - ept).Len()
    bar, lam, bGoRight = tctright if bGoRight else tctleft
    res = [ ]
    while bar != ebarlam[0]:
        res.append((bar, lam))
        bar, lam, bGoRight = TriangleCrossTowards(bar, lam, bGoRight, ebarlam[0], ept)
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
for i in range(1, len(drivepts)-1):
    pt = drivepts[i]
    ebarlam = utbm.FindClosestEdge(P3(*pt))
    if drivebars[-1][0] == ebarlam[0]:
        print("skipping samebar (sameproj at)", drivebars[-1][1], ebarlam[1])
        continue
    drivebars.extend(BetweenBarListE(drivebars[-1], ebarlam))
    drivebars.append(ebarlam)
drivebars.extend(BetweenBarListE(drivebars[-1], startbarlam))
drivebars.append(startbarlam)

epts = [ ]
for bar, lam in drivebars:
    ept = Along(lam, bar.nodeback.p, bar.nodefore.p)
    epts.append(ept)
Part.show(Part.makePolygon(epts))


def facetbetweenbars(bar, barN):
    assert bar != barN
    nodeR = bar.barforeright.GetNodeFore(bar.barforeright.nodeback == bar.nodefore)
    if barN.nodeback == nodeR or barN.nodefore == nodeR:
        if nodeR.i > bar.nodeback.i:
            tbar = bar
        else:
            tbar = bar.barforeright.GetForeRightBL(bar.barforeright.nodeback == bar.nodefore)
            assert tbar.nodefore == bar.nodeback
    else:
        nodeL = bar.barbackleft.GetNodeFore(bar.barbackleft.nodeback == bar.nodeback)
        if not (barN.nodeback == nodeL or barN.nodefore == nodeL):
            print("  b:", bar.nodeback, bar.nodefore)
            print("  N:", barN.nodeback, barN.nodefore)
        assert barN.nodeback == nodeL or barN.nodefore == nodeL
        if bar.nodeback.i > nodeL.i:
            tbar = bar.barbackleft.GetForeRightBL(bar.barbackleft.nodeback == bar.nodeback)
        else:
            tbar = bar.barbackleft
            
    DtbarR = tbar.barforeright.GetNodeFore(tbar.barforeright.nodeback == tbar.nodefore)
    assert DtbarR.i > tbar.nodeback.i
    Dtbars = [ tbar, tbar.barforeright, 
               tbar.barforeright.GetForeRightBL(tbar.barforeright.nodeback == tbar.nodefore) ]
    assert bar in Dtbars and barN in Dtbars
    return tbar
    	
tbarfacets = [ facetbetweenbars(drivebars[i][0], drivebars[i+1][0])  for i in range(len(drivebars)-1) ]
facets = [ [ Vector(*tbar.nodeback.p), Vector(*tbar.nodefore.p), 
             Vector(*TriangleNodeOpposite(tbar, True).p) ]  for tbar in tbarfacets ]
mesh = doc.addObject("Mesh::Feature", "m1")
mesh.Mesh = Mesh.Mesh(facets)




