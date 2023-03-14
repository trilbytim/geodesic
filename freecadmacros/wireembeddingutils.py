from barmesh.tribarmes import TriangleBarMesh, TriangleBar, MakeTriangleBoxing
from barmesh.basicgeo import I1, Partition1, P3, P2, Along

def TriangleCrossCutPlane(bar, lam, bGoRight, driveperpvec, driveperpvecDot):
    nodeAhead = bar.GetNodeFore(bGoRight)
    nodeBehind = bar.GetNodeFore(not bGoRight)
    barAhead = bar.GetForeRightBL(bGoRight)
    barAheadGoRight = (barAhead.nodeback == nodeAhead)
    nodeOpposite = barAhead.GetNodeFore(barAheadGoRight)
    barBehind = barAhead.GetForeRightBL(barAheadGoRight)
    DbarBehindGoRight = (barBehind.nodeback == nodeOpposite)
    assert nodeBehind == barBehind.GetNodeFore(DbarBehindGoRight)
    dpvdAhead = P3.Dot(driveperpvec, nodeAhead.p)
    dpvdBehind = P3.Dot(driveperpvec, nodeBehind.p)
    dpvdOpposite = P3.Dot(driveperpvec, nodeOpposite.p)
    assert dpvdAhead > driveperpvecDot - 0.001 and dpvdBehind < driveperpvecDot + 0.001
    bAheadSeg = (dpvdOpposite < driveperpvecDot)
    barCrossing = (barAhead if bAheadSeg else barBehind)
    dpvdAB = (dpvdAhead if bAheadSeg else dpvdBehind)
    barCrossingLamO = -(dpvdOpposite - driveperpvecDot)/(dpvdAB - dpvdOpposite)
    assert barCrossingLamO >= 0.0
    barCrossingLam = barCrossingLamO if (barCrossing.nodeback == nodeOpposite) else 1-barCrossingLamO
    barCrossingGoRight = (barCrossing.nodeback == nodeOpposite) == bAheadSeg
    return barCrossing, barCrossingLam, barCrossingGoRight

def planecutembeddedcurve(startbar, startlam, driveperpvec):
    startpt = Along(startlam, startbar.nodeback.p, startbar.nodefore.p)
    driveperpvecDot = P3.Dot(driveperpvec, startpt)
    drivebars = [ (startbar, startlam) ]
    bGoRight = (P3.Dot(driveperpvec, startbar.nodefore.p - startbar.nodeback.p) > 0)
    bar, lam = startbar, startlam
    while True:
        bar, lam, bGoRight = TriangleCrossCutPlane(bar, lam, bGoRight, driveperpvec, driveperpvecDot)
        drivebars.append((bar, lam))
        if bar == startbar:
            assert abs(startlam - lam) < 0.001
            drivebars[-1] = drivebars[0]#
            break
        if len(drivebars) > 500:
            print("failed, too many drivebars")
            break
    return drivebars

def showdrivebarsmesh(drivebars, doc, meshname="m1"):
    def facetnoderight(bar):
        return bar.barforeright.GetNodeFore(bar.barforeright.nodeback == bar.nodefore)
    tbarfacets = [ facetbetweenbars(drivebars[i][0], drivebars[i+1][0])  for i in range(len(drivebars)-1) ]
    facets = [ [ Vector(*tbar.nodeback.p), Vector(*tbar.nodefore.p), 
                 Vector(*facetnoderight(tbar).p) ]  for tbar in tbarfacets ]
    mesh = doc.addObject("Mesh::Feature", meshname)
    mesh.Mesh = Mesh.Mesh(facets)

def showdrivebarscurve(drivebars, doc):
    epts = [ Along(lam, bar.nodeback.p, bar.nodefore.p)  for bar, lam in drivebars ]
    Part.show(Part.makePolygon(epts))

