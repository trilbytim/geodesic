
from FreeCAD import Vector
import Draft, Part, Mesh, MeshPart
from PySide import QtGui ,QtCore

doc = None
sel = [ ]

def init(App):
    global doc, sel
    doc = App.ActiveDocument
    sel = App.Gui.Selection.getSelection()

def qrow(qw, slab, yval):
    v = QtGui.QLineEdit(qw); 
    v.move(120, yval); 
    vlab = QtGui.QLabel(slab, qw)
    vlab.move(20, yval+5)
    return v

def getlabelofselectedwire():
    for s in sel:
        if hasattr(s, "Shape") and isinstance(s.Shape, Part.Wire):
            return s.Label
    return ""
    
def getlabelofselectedmesh():
    for s in sel:
        if hasattr(s, "Mesh") and isinstance(s.Mesh, Mesh.Mesh):
            return s.Label
    return ""

def getlabelofselectedsketch():
    for s in sel:
        if hasattr(s, "Module") and s.Module == 'Sketcher':
            return s.Label
    return ""
    
def findobjectbylabel(lab):
    objs = [ obj  for obj in doc.findObjects(Label=lab)  if obj.Label == lab ]
    return objs[0] if objs else None

def removeobjectbylabel(lab):
    if lab:
        objs = doc.findObjects(Label=lab)
        for obj in objs:
            if obj.Label == lab:
                doc.removeObject(obj.Name)



from barmesh.basicgeo import I1, Partition1, P3, P2, Along
from trianglemeshutils import facetbetweenbars

def showdrivebarscurve(drivebars, lab):
    removeobjectbylabel(lab)
    epts = [ Along(lam, bar.nodeback.p, bar.nodefore.p)  for bar, lam in drivebars ]
    wire = Part.show(Part.makePolygon(epts), lab)
    return wire


def showdrivebarsmesh(drivebars, lab):
    removeobjectbylabel(lab)
    def facetnoderight(bar):
        return bar.barforeright.GetNodeFore(bar.barforeright.nodeback == bar.nodefore)
    tbarfacets = [ facetbetweenbars(drivebars[i][0], drivebars[i+1][0])  for i in range(len(drivebars)-1) ]
    facets = [ [ Vector(*tbar.nodeback.p), Vector(*tbar.nodefore.p), 
                 Vector(*facetnoderight(tbar).p) ]  for tbar in tbarfacets ]
    mesh = doc.addObject("Mesh::Feature", lab)
    mesh.Mesh = Mesh.Mesh(facets)
    return mesh
