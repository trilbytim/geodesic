# -*- coding: utf-8 -*-
# Embed a curve into a mesh so we can head off in different directions and tell when it is crossed

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 

import os, sys, math, time
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])

from barmesh.basicgeo import I1, Partition1, P3, P2, Along
from curvesutils import isdiscretizableobject, discretizeobject
from trianglemeshutils import UsefulBoxedTriangleMesh, facetbetweenbars
from wireembeddingutils import TriangleCrossCutPlane, planecutembeddedcurve

doc = App.ActiveDocument
gui = App.Gui
sel = Gui.Selection.getSelection()

# drive curve definition (cut plane)
drivept0 = P3(110.160362, -1.82, 1.1216061)
driveperpvec = P3(0, 1, 0)

meshobject = None
for s in sel:
    if hasattr(s, "Mesh") and isinstance(s.Mesh, Mesh.Mesh):
        meshobject = s.Mesh
if not meshobject:
    print("Need a mesh object selected")

utbm = UsefulBoxedTriangleMesh(meshobject)
startbar, startlam = utbm.FindClosestEdge(drivept0)
drivebars = planecutembeddedcurve(startbar, startlam, driveperpvec)

epts = [ Along(lam, bar.nodeback.p, bar.nodefore.p)  for bar, lam in drivebars ]
Part.show(Part.makePolygon(epts))

def facetnoderight(bar):
    return bar.barforeright.GetNodeFore(bar.barforeright.nodeback == bar.nodefore)
tbarfacets = [ facetbetweenbars(drivebars[i][0], drivebars[i+1][0])  for i in range(len(drivebars)-1) ]
facets = [ [ Vector(*tbar.nodeback.p), Vector(*tbar.nodefore.p), 
             Vector(*facetnoderight(tbar).p) ]  for tbar in tbarfacets ]
mesh = doc.addObject("Mesh::Feature", "m1")
mesh.Mesh = Mesh.Mesh(facets)




