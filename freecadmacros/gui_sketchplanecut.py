# -*- coding: utf-8 -*-
# Use the MeshPart library to project a selected wire onto a selected mesh

import Draft, Part, Mesh, MeshPart
from barmesh.basicgeo import I1, Partition1, P3, P2, Along
from FreeCAD import Vector, Rotation 
from PySide import QtGui, QtCore


import sys;  sys.modules.pop("wireembeddingutils")
from wireembeddingutils import planecutembeddedcurve, planecutbars
from trianglemeshutils import UsefulBoxedTriangleMesh, facetbetweenbars
#import sys;  sys.modules.pop("freecadutils")

import freecadutils
freecadutils.init(App)

def okaypressed():
    print("Okay Pressed") 
    sketchplane = freecadutils.findobjectbylabel(qsketchplane.text())
    meshobject = freecadutils.findobjectbylabel(qmeshobject.text())
    if sketchplane and meshobject:
        driveperpvec = sketchplane.Placement.Rotation.multVec(Vector(0,0,1))
        driveperpvecDot = driveperpvec.dot(sketchplane.Placement.Base)
        utbm = UsefulBoxedTriangleMesh(meshobject.Mesh)
        startbar, startlam = planecutbars(utbm.tbarmesh, driveperpvec, driveperpvecDot)
        drivebars = planecutembeddedcurve(startbar, startlam, driveperpvec)
        if qcutwire.text():
            freecadutils.showdrivebarscurve(drivebars, qcutwire.text())
        if qcutwiresnake.text():
            freecadutils.showdrivebarsmesh(drivebars, qcutwiresnake.text())
    else:
        print("Need to select a Sketch and a Mesh object in the UI to make this work")
    print("qoptioncheck", qoptioncheck.isChecked())
    qw.hide()

qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 400, 300, 200)
qw.setWindowTitle('Cut sketch plane')
qsketchplane = freecadutils.qrow(qw, "Sketchplane: ", 15)
qmeshobject = freecadutils.qrow(qw, "Meshobject: ", 50)
qcutwire = freecadutils.qrow(qw, "Cut wire: ", 85)
qcutwiresnake = freecadutils.qrow(qw, "Cut wire snake: ", 115)
okButton = QtGui.QPushButton("Cut plane", qw)
okButton.move(160, 150)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  

qsketchplane.setText(freecadutils.getlabelofselectedsketch())
qmeshobject.setText(freecadutils.getlabelofselectedmesh())
qcutwire.setText("planecutwire")
qcutwiresnake.setText("planecutwiresnake")

qw.show()


    


