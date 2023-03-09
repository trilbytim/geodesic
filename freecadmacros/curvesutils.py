
from FreeCAD import Vector

def isdiscretizableobject(s):
    if s.isDerivedFrom("Sketcher::SketchObject"):
        print("we could do joining up of pieces by coincidence here")
        assert hasattr(s, "Shape") and hasattr(s.Shape, "Edges")
        return True
    elif hasattr(s, "Shape") and hasattr(s.Shape, "Edges"):
	    return True
    return False

def discretizeobject(s, deflection=0.02):
    pts = [ ]
    for edge in s.Shape.Edges:
        epts = edge.discretize(Deflection=deflection)
        gap = (pts[-1] - epts[0]).Length if pts else 0
        if gap > 0.1:
            print("Gap between edges", gap)
        pts.extend(epts[1 if gap > 0.1 else 0:])
    return pts


def thinptstotolerance(pts, tol):
    i0 = 0
    istack = [ len(pts)-1 ]
    ithinned = [ i0 ]
    while istack:
        i1 = istack[-1]
        p0, p1 = pts[i0], pts[i1]
        v = p1 - p0
        vsq = v.Lensq()
        imid = -1
        dmid = 0.0
        for i in range(i0+1, i1):
            p = pts[i]
            lam = P3.Dot(v, (p - p0))/vsq
            lam = max(0.0, min(1.0, lam))
            pm = p0 + v*lam
            d = (pm - p).Len()
            if imid == -1 or d > dmid:
                imid = i
                dmid = d
        if dmid <= tol:
            i0 = istack.pop()
            ithinned.append(i0)
        else:
            assert imid != -1
            istack.append(imid)
    return [ pts[i]  for i in ithinned ]

