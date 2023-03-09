# -*- coding: utf-8 -*-
# Use the MeshPart library to project a selected wire onto a selected mesh

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 
import math, numpy

doc = App.ActiveDocument
gui = App.Gui
sel = Gui.Selection.getSelection()

rad = 100
th = numpy.linspace(0, 360, 40)
rth = numpy.radians(th)
crth = numpy.cos(rth)
srth = numpy.sin(rth)

phi = (1 + math.sqrt(5))/2
cornervecs = [ 
    Vector(1,1,1), Vector(1,-1,1), Vector(-1,-1,1), Vector(-1,1,1), 
    Vector(0,phi,1/phi), Vector(0,phi,-1/phi), 
    Vector(1/phi,0,phi), Vector(1/phi,0,-phi),
    Vector(phi,1/phi,0), Vector(phi,-1/phi,0) ]
cornervecs = [ lcvec.normalize()  for lcvec in cornervecs ]

def perpvecs(cvec):
	xvec = cvec.cross(Vector(0,0,1)).normalize()
	yvec = cvec.cross(xvec)
	return xvec, yvec

for cvec in cornervecs:
	xvec, yvec = perpvecs(cvec)
	gpts = [ ]
	for c, s in zip(crth, srth):
		gpts.append(xvec*c*rad + yvec*s*rad)
#	Part.show(Part.makePolygon(gpts))

for i in range(len(cornervecs)):
    cvec = cornervecs[i]
    xvec, yvec = perpvecs(cvec)
    angs = [ ]
    for j in range(len(cornervecs)):
         if i != j:
             ocvec = cornervecs[j]
             avec = cvec.cross(ocvec)
             ang = math.degrees(math.atan2(xvec.dot(avec), yvec.dot(avec))) + 180
             angs.append(ang)
             angs.append((ang + 180) % 360)
    angs.sort()
    print(" ".join("%.0f" % (ang - angs[0])  for ang in angs))
    print("  ", " ".join("%.4f" % ((angs[(i+1)%len(angs)] - angs[i])/360)  for i in range(len(angs))))
    print()
    
