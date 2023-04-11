# -*- coding: utf-8 -*-
# Use the MeshPart library to project a selected wire onto a selected mesh

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 

from PySide import QtGui ,QtCore


import Part, FreeCAD
from FreeCAD import Base,Vector

doc = App.ActiveDocument
gui = App.Gui
sel = Gui.Selection.getSelection()

screenrot = Gui.ActiveDocument.ActiveView.getCameraOrientation()
projectionDir = screenrot.multVec(Vector(0,0,-1))

def qrow(qw, slab, yval):
    v = QtGui.QLineEdit(qw); 
    v.move(120, yval); 
    vlab = QtGui.QLabel(slab, qw)
    vlab.move(20, yval+5)
    return v

def okaypressed():
    print("Okay Pressed") 
    wireobject = doc.getObject(qwireshape.text())
    meshobject = doc.getObject(qmeshobject.text())
    if wireobject and meshobject:
        polylines = MeshPart.projectShapeOnMesh(wireobject.Shape, meshobject.Mesh, projectionDir)
        for i in polylines:
            Part.show(Part.makePolygon(i))
    else:
        print("Need to select a Wire and a Mesh object in the UI to make this work")
    qw.hide()

qw = QtGui.QWidget()
qw.setGeometry(700, 400, 300, 200)
qw.setWindowTitle('Project path')
qwireshape = qrow(qw, "Wireshape: ", 15)
qmeshobject = qrow(qw, "Meshobject: ", 50)
okButton = QtGui.QPushButton("Project Wire", qw)
okButton.move(160, 150)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  
for s in sel:
    if hasattr(s, "Shape") and isinstance(s.Shape, Part.Wire):
        qwireshape.setText(s.Name)
    if hasattr(s, "Mesh") and isinstance(s.Mesh, Mesh.Mesh):
        qmeshobject.setText(s.Name)
qw.show()



    


