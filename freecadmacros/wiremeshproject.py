# -*- coding: utf-8 -*-
# Use the MeshPart library to project a selected wire onto a selected mesh

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 

doc = App.ActiveDocument
gui = App.Gui
sel = Gui.Selection.getSelection()

screenrot = Gui.ActiveDocument.ActiveView.getCameraOrientation()
projectionDir = screenrot.multVec(Vector(0,0,-1))

meshobject = None
wireshape = None

for s in sel:
    if hasattr(s, "Shape") and isinstance(s.Shape, Part.Wire):
        wireshape = s.Shape
    if hasattr(s, "Mesh") and isinstance(s.Mesh, Mesh.Mesh):
        meshobject = s.Mesh

if wireshape and meshobject:
    polylines = MeshPart.projectShapeOnMesh(wireshape, meshobject, projectionDir)
    for i in polylines:
        Part.show(Part.makePolygon(i))
else:
    print("Need to select a Wire and a Mesh object in the UI to make this work")
    
