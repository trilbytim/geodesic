# -*- coding: utf-8 -*-
# Use the Barmesh system to run a geodesic line continuing from the start of 
# the selected wire path

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 

import os, sys
sys.path.append(os.path.join(os.path.split(__file__)[0]))

from barmesh.tribarmes import TriangleBarMesh, TriangleBar, MakeTriangleBoxing
from barmesh.basicgeo import I1, Partition1, P3, P2, Along
from FreeCAD import Vector

doc = App.ActiveDocument
gui = App.Gui
sel = Gui.Selection.getSelection()

screenrot = Gui.ActiveDocument.ActiveView.getCameraOrientation()
projectionDir = screenrot.multVec(Vector(0,0,-1))

tbarmesh = None
tboxing = None
hitreg = None
nhitreg = 0

def clamp(num, min_value, max_value):
    return max(min(num, max_value), min_value)
        
def maketbarmesh(meshobject):
    global tbarmesh, tboxing, hitreg, nhitreg
    trpts = [ sum(f.Points, ())  for f in meshobject.Facets ]
    print("building tbarmesh ", len(trpts))
    tbarmesh = TriangleBarMesh()
    tbarmesh.BuildTriangleBarmesh(trpts)
    assert len(tbarmesh.nodes) == meshobject.CountPoints
    assert len(tbarmesh.bars) == meshobject.CountEdges
    tboxing = MakeTriangleBoxing(tbarmesh)
    hitreg = [0]*len(tbarmesh.bars)




def Square(X):
    return X*X
def TOL_ZERO(X):
    if not (abs(X) < 0.0001):
        print("TOL_ZERO fail", X)

def GeoCrossAxisE(a, Vae, Vab, Isq, Isgn):
    # Solve: Isq*x.Lensq() - Square(P3.Dot(x, Vab)) = 0   for x = a + Vae*q
    # 0 = Isq*(a^2 + 2q a.Vae + q^2 Vae^2) - (a.Vab + Vae.Vab q)^2
    #   = Isq*(a^2 + 2q adf + q^2 Vae^2) - (adv + fdv q)^2

    fdv = P3.Dot(Vae, Vab)
    adv = P3.Dot(a, Vab)
    adf = P3.Dot(a, Vae)
    qA = Square(fdv) - Vae.Lensq()*Isq
    qB2 = adv*fdv - adf*Isq
    qC = Square(adv) - a.Lensq()*Isq
    if abs(qA) <= abs(qB2)*1e-7:
        if qB2 == 0:
            return -1.0
        q = -qC/(2*qB2)
    else:
        qdq = Square(qB2) - qA*qC
        if qdq < 0.0:
            #print("qdq", qdq)
            return -1.0
        qs = math.sqrt(qdq) / qA
        qm = -qB2 / qA
        q = qm + qs*Isgn
    # q = qs +- qm,  x = a + Vae*q,  Dot(x, Vab) same sign as Dot(Vcd, Vab)
    if abs(q) < 100:
        TOL_ZERO(qA*Square(q) + qB2*2*q + qC)
    return q

#
# This is the basic function that crosses from one triangle to the next
#
def GeoCrossAxis(Ga, Gb, Gc, lam, Ge):
    Vab = Gb - Ga
    Gd = Ga + Vab*lam
    Vcd = Gd - Gc
    if Vcd.Len() == 0:
        bEnd = True
        bAEcrossing = False
        q = 0
        Gx = Gc
    else:
        bEnd = False
        cdDab = P3.Dot(Vcd, Vab)
        Isq = Square(cdDab) / Vcd.Lensq()
        Isgn = -1 if cdDab < 0 else 1
        qVae = GeoCrossAxisE(Ga - Gd, Ge - Ga, Vab, Isq, Isgn)
        qVbe = GeoCrossAxisE(Gb - Gd, Ge - Gb, -Vab, Isq, -Isgn)
        bAEcrossing = (abs(qVae - 0.5) < abs(qVbe - 0.5))
        q = qVae if bAEcrossing else qVbe
        Gx = (Ga + (Ge - Ga)*q) if bAEcrossing else (Gb + (Ge - Gb)*q)
        Dx = Gx - Gd
        TOL_ZERO(Isq - Square(P3.Dot(Dx, Vab)/Dx.Len()))
        TOL_ZERO(P3.Dot(Vcd, Vab)/Vcd.Len() - P3.Dot(Dx, Vab)/Dx.Len())
    return bAEcrossing, q, Gx, bEnd

# This is the iterative function that goes from bar to bar through the mesh
# c=from point, bar the crossing edge, lam the point on the bar, and bGoRight is the crossing direction
#

def TriangleNodeOpposite(bar, bGoRight):
    bartop = bar.GetForeRightBL(bGoRight)
    if bartop != None:
        return bartop.GetNodeFore(bartop.nodeback == bar.GetNodeFore(bGoRight))
    return None
    
def GeoCrossBar(c, bar, lam, bGoRight):
    Na, Nb = bar.nodeback, bar.nodefore
    d = Na.p + (Nb.p - Na.p)*lam
    Ne = TriangleNodeOpposite(bar, bGoRight)
    if Ne == None:
        return (None, None, 0.0, False)
    print(Na.p, Nb.p, Ne.p)
    bAEcrossing, q, Gx, bEnd = GeoCrossAxis(Na.p, Nb.p, c, lam, Ne.p)
    if bGoRight:
        if bAEcrossing:
            bar = bar.barforeright.GetForeRightBL(bar.barforeright.nodeback == Nb)
            lam = q if bar.nodeback == Na else 1-q
            bGoRight = (bar.nodeback == Na)   
        else:
            bar = bar.barforeright
            lam = q if bar.nodeback == Nb else 1-q
            bGoRight = not (bar.nodeback == Nb)
    else:
        if bAEcrossing:
            bar = bar.barbackleft
            lam = q if bar.nodeback == Na else 1-q
            bGoRight = not (bar.nodeback == Na)
        else:
            bar = bar.barbackleft.GetForeRightBL(bar.barbackleft.nodeback == Na)
            lam = q if bar.nodeback == Nb else 1-q
            bGoRight = (bar.nodeback == Nb)
            
    c = bar.nodeback.p + (bar.nodefore.p - bar.nodeback.p)*lam
    TOL_ZERO((c - Gx).Len())
    return (d, bar, lam, bGoRight)


def FindClosestEdge(p, r=3):
    global nhitreg
    nhitreg += 1
    barclosest = None
    barclosestlam = 0.0
    for ix, iy in tboxing.CloseBoxeGenerator(p.x, p.x, p.y, p.y, r):
        tbox = tboxing.boxes[ix][iy]
        for i in tbox.edgeis:
            if hitreg[i] != nhitreg:
                bar = tbarmesh.bars[i]
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
                hitreg[i] = nhitreg
    return barclosest, barclosestlam

meshobject = None
wireshape = None

for s in sel:
    if hasattr(s, "Shape") and isinstance(s.Shape, Part.Wire):
        wireshape = s.Shape
    if hasattr(s, "Mesh") and isinstance(s.Mesh, Mesh.Mesh):
        meshobject = s.Mesh
	
if meshobject:
    maketbarmesh(meshobject)

if wireshape and meshobject:
    pts = [ v.Point  for v in wireshape.Vertexes ]
    print("making the closest point stuff")
    p0 = P3(*pts[0])
    c = P3(*pts[1])
    bar, lam = FindClosestEdge(p0)
    Neright = TriangleNodeOpposite(bar, True)
    Neleft = TriangleNodeOpposite(bar, False)
    bGoRight = (P3.Dot(Neright.p - Neleft.p, p0 - c) > 0.0)
    gpts = [ ]
    for i in range(500):
        c, bar, lam, bGoRight = GeoCrossBar(c, bar, lam, bGoRight)
        gpts.append(Vector(*c))
        if bar == None:
            break
    Part.show(Part.makePolygon(gpts))

else:
    print("Need to select a Wire and a Mesh object in the UI to make this work")
 
