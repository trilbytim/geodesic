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

cornervecs = [ Vector(1,1,1).normalize(), Vector(1,-1,1).normalize(), 
			   Vector(-1,-1,1).normalize(), Vector(-1,1,1).normalize() ]


for cvec in cornervecs:
	xvec = cvec.cross(Vector(0,0,1)).normalize()
	yvec = cvec.cross(xvec)
	gpts = [ ]
	for c, s in zip(crth, srth):
		gpts.append(xvec*c*rad + yvec*s*rad)
	Part.show(Part.makePolygon(gpts))

