# -*- coding: utf-8 -*-
# Use the MeshPart library to project a selected wire onto a selected mesh

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 

from curvesutils import isdiscretizableobject, discretizeobject, thinptstotolerance
from barmesh.basicgeo import I1, Partition1, P3, P2, Along
import math

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
    robotstate["E1"] = prevE1
    robotstate["E3"] = prevE3
    return res

def srcpt(ps):
	return "{%s}" % ", ".join("%s %+9.3f" % (c, ps[c])  for c in ["X", "Y", "Z", "A", "E1", "E3", "E4"]  if c in ps)
		
fname = "testAAA.src"
headersrc = os.path.join(os.path.split(__file__)[0], "header.src")
print("making ", os.path.abspath(fname), " from ", __file__)
freetapelength = 30.0
selwireshapes = [ ]

for s in sel:
    if isdiscretizableobject(s):
        drivepts = discretizeobject(s, deflection=0.2)
if not drivepts:
    print("Need a linearizable object selected")

dpts = [ P3(*p)  for p in drivepts ]
tdpts = thinptstotolerance(dpts, tol=0.2)
print("tolthin", len(dpts), len(tdpts))

rotzadvances = [ 0, 199.12638313124705 ] 

def rotxz(pt, vrotX):
    rptxz = vrotX*pt.x + P2.APerp(vrotX)*pt.z
    return P3(rptxz.u, pt.y, rptxz.v)

Dcheckpath = [ ]

fout = open(fname, "w")
fout.write(open(headersrc).read())
robotstate = { "X":0, "Y":300, "Z":0, "A":0, "E1":0, "E3":0, "E4":0 }
fout.write("SLIN %s\n" % srcpt(robotstate))
for rotzadvanceby in rotzadvances:
    fout.write("SPLINE\n")
    vrotX = P2(math.cos(math.radians(rotzadvanceby)), math.sin(math.radians(rotzadvanceby)))
    rtdpts = [ rotxz(pt, vrotX)  for pt in tdpts ]
    lns = outputsrclines(rtdpts, freetapelength, robotstate)
    Dcheckpath.extend(rtdpts)
    for ln in lns:
        fout.write("SPL %s\n" % srcpt(ln))
    fout.write("ENDSPLINE\n")
fout.write("HALT\nEND\n")
fout.close()    
print("path made")

Part.show(Part.makePolygon([Vector(*p)  for p in Dcheckpath]))

