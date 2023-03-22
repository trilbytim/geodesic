# -*- coding: utf-8 -*-
# Use the MeshPart library to project a selected wire onto a selected mesh

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 

from curvesutils import isdiscretizableobject, discretizeobject, thinptstotolerance
from barmesh.basicgeo import I1, Partition1, P3, P2, Along
import math, numpy

doc = App.ActiveDocument
gui = App.Gui
sel = Gui.Selection.getSelection()

# X - moves towards/away from the tool (perpendicular to the carriage).
# Y - moves along the carriage axis
# Z - Vertical positioning (rarely used)
# A - Yaw of the robot (around the Z axis)
# E1 - Rotation of the payout eye
# E3 - Rotation both chucks (headstock and tailstock)
# E4 - Rotation of tailstock relative to headstock (rarely used)

def outputsrclines(pts, freetapelength, robotstate):
    prevE1 = robotstate["E1"]  # used to track the number of complete winds on the eyelet part
    prevE3 = robotstate["E3"]  # used to track the number of complete winds on the lathe part
    res = [ ]

    # loop through the points and vectors 
    # (we use the incoming vector at each triangle edge, not the outgoing vector)
    for i in range(1, len(pts)):
        pt = pts[i]
        vec = P3.ZNorm(pts[i] - pts[i-1])
        tapevector = vec*freetapelength
        tcp = pt + tapevector
        radialvec = P2(tcp.x, tcp.z)
        lE3 = radialvec.Arg()
        E3 = lE3 + 360*int((abs(prevE3 - lE3)+180)/360)*(1 if prevE3 > lE3 else -1)
        X = -radialvec.Len()
        tangentunitvec = -P2.CPerp(radialvec)*(1/X)
        E1vec = P2(-P2.Dot(tangentunitvec, P2(tapevector.x, tapevector.z)), -tapevector.y)
        lE1 = E1vec.Arg()
        E1 = lE1 + 360*int((abs(prevE1 - lE1)+180)/360)*(1 if prevE1 > lE1 else -1)
        res.append({"X":X, "Y":tcp.y, "E1":E1, "E3":E3})
        prevE1 = E1
        prevE3 = E3
        robotstate["Y"] = tcp.y
        robotstate["X"] = X
    robotstate["E1"] = prevE1
    robotstate["E3"] = prevE3

    return res

def srcpt(ps):
	return "{%s}" % ", ".join("%s %+9.3f" % (c, ps[c])  for c in ["X", "Y", "Z", "A", "E1", "E3", "E4"]  if c in ps)

		
fname = "filwin9e.src"
headersrc = os.path.join(os.path.split(__file__)[0], "header.src")
print("making ", os.path.abspath(fname), " from ", __file__)
selwireshapes = [ ]


tcpcurve = None
tapecurve = None
for s in sel:
    if isdiscretizableobject(s):
        pts = [ P3(*p)  for p in discretizeobject(s, deflection=0.02) ]
        rg = I1.AbsorbList(pt[2]  for pt in pts)
        if rg.lo == rg.hi == 0:
            tcpcurve = pts
        else:
            assert tapecurve is None, "two tape curves found"
            tapecurve = thinptstotolerance(pts, tol=0.2)
            tapecurve = pts
            
if not tapecurve:
    print("Need a linearizable tape curve object selected")
if not tcpcurve:
    print("Need a linearizable tool centre point (tcp) curve object selected")

import scipy.interpolate
tcpinterpolator = scipy.interpolate.CubicSpline([p.y for p in tcpcurve], [p.x for p in tcpcurve], axis=0, bc_type='natural')
tcpsplineys = numpy.linspace(tcpcurve[0].y, tcpcurve[-1].y, 200)
tcpsplinexs = tcpinterpolator(tcpsplineys)
#Part.show(Part.makePolygon([Vector(x, y, 0)  for x, y in zip(tcpsplinexs, tcpsplineys)]))
#Part.show(Part.makePolygon([Vector(*p)  for p in tcpcurve]))
tcpconstXval = min(p.x for p in tcpcurve)

def projectToXvalplane(pt, vec, cx):
    qa = P2(vec.x, vec.z).Lensq()
    qb2 = pt.x*vec.x + pt.z*vec.z
    qc = P2(pt.x, pt.z).Lensq() - cx*cx
    qdq = qb2*qb2 - qa*qc
    qs = math.sqrt(qdq) / qa
    qm = -qb2 / qa
    q = qm + qs
    #TOL_ZERO((P2(pt.x, pt.z) + P2(vec.x, vec.z)*q).Len())
    return pt + vec*q

print("tcpconstXval", tcpconstXval, len(tapecurve))
tapevecs = [ ]
tapetcps = [ ]

facets = [ ]
#for i in range(1, len(tapecurve)):
for i in range(470, 550):
    vec = tapecurve[i] - tapecurve[i-1]
    tcp = projectToXvalplane(tapecurve[i], vec, tcpconstXval)
    if tapetcps:
        facets.append([Vector(*tapetcps[-1]), Vector(*tapecurve[i-1]), Vector(*tcp)])
        facets.append([Vector(*tcp), Vector(*tapecurve[i-1]), Vector(*tapecurve[i])])
    tapevecs.append(vec)
    tapetcps.append(tcp)
Part.show(Part.makePolygon([Vector(*p)  for p in tapetcps]))
mesh = doc.addObject("Mesh::Feature", "tcpmesh")
mesh.Mesh = Mesh.Mesh(facets)

# need to convert the curve into a lookup XY thing
# then we project out until the radius is X at the position Y
# From here we can rotate round to fit the E values

# outputsrclines will take tcp and tapevector
# we will sample along this fixed X (along Y) path 
# and break the splines at the extreme Y points
# We can make a tcpconstXval too

# Somewhere we need to convexify the path.  Cannot have the vector out 
# along the tape intersecting the mesh (which happens when there is concavity.
# We can trim back on the geodesic where the back points are discarded until it's convex

