# -*- coding: utf-8 -*-

# directional geodesics from embedded curve

# Embed a curve into a mesh so we can head off in different directions and tell when it is crossed

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 
from PySide import QtGui, QtCore

import os, sys, math, time
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])

from barmesh.basicgeo import I1, Partition1, P3, P2, Along
from curvesutils import isdiscretizableobject, discretizeobject
from curvesutils import cumlengthlist, seglampos
from trianglemeshutils import UsefulBoxedTriangleMesh, facetbetweenbars
from wireembeddingutils import planecutembeddedcurve, planecutbars

import sys;  sys.modules.pop("geodesicutils")
from geodesicutils import drivegeodesic, InvAlong

import freecadutils
freecadutils.init(App)

def okaypressed():
    print("Okay Pressed") 
    sketchplane = freecadutils.findobjectbylabel(qsketchplane.text())
    meshobject = freecadutils.findobjectbylabel(qmeshobject.text())
    alongwire = float(qalongwire.text())
    dsangle = float(qanglefilament.text())
    if sketchplane and meshobject:
        driveperpvec = sketchplane.Placement.Rotation.multVec(Vector(0,0,1))
        driveperpvecDot = driveperpvec.dot(sketchplane.Placement.Base)
        utbm = UsefulBoxedTriangleMesh(meshobject.Mesh)
        startbar, startlam = planecutbars(utbm.tbarmesh, driveperpvec, driveperpvecDot)
        drivebars = planecutembeddedcurve(startbar, startlam, driveperpvec)
        tridrivebarsmap = dict((facetbetweenbars(drivebars[dseg][0], drivebars[dseg+1][0]).i, dseg)  for dseg in range(len(drivebars)-1))

        dpts = [ Along(lam, bar.nodeback.p, bar.nodefore.p)  for bar, lam in drivebars ]
        dptcls = cumlengthlist(dpts)

        ds = Along(alongwire, dptcls[0], dptcls[-1])
        gbs1, ds1, dsangle1 = drivegeodesic(drivebars, tridrivebarsmap, dpts, dptcls, ds, dsangle)
        print("pos ds1", ds1, dsangle1)
        gbs2 = [ gbs1[-1] ]
        if ds1 != -1:
            gbs2, ds2, dsangle2 = drivegeodesic(drivebars, tridrivebarsmap, dpts, dptcls, ds1, dsangle1+0)
        Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbs1+gbs2[1:]]), qoutputfilament.text())
        if ds1 != -1 and ds2 != -1:
            qalongwire.setText("%f" % InvAlong(ds2, dptcls[0], dptcls[-1]))
            qanglefilament.setText("%f" % dsangle2)
            print("Cylinder position angle advance degrees", 360*(ds2 - ds)/dptcls[-1])
            print("Leaving angle", dsangle, "Continuing angle", dsangle2)
            try:
                qoutputfilament.setText("w%d" % (int(qoutputfilament.text()[1:]) + 1))
            except ValueError:
                pass
    else:
        print("Need to select a Sketch and a Mesh object in the UI to make this work")
        qw.hide()



def spraylines():
    sketchplane = freecadutils.findobjectbylabel(qsketchplane.text())
    meshobject = freecadutils.findobjectbylabel(qmeshobject.text())
    alongwire = float(qalongwire.text())
    dsangle = float(qanglefilament.text())
    if sketchplane and meshobject:
        driveperpvec = sketchplane.Placement.Rotation.multVec(Vector(0,0,1))
        driveperpvecDot = driveperpvec.dot(sketchplane.Placement.Base)
        utbm = UsefulBoxedTriangleMesh(meshobject.Mesh)
        startbar, startlam = planecutbars(utbm.tbarmesh, driveperpvec, driveperpvecDot)
        drivebars = planecutembeddedcurve(startbar, startlam, driveperpvec)
        tridrivebarsmap = dict((facetbetweenbars(drivebars[dseg][0], drivebars[dseg+1][0]).i, dseg)  for dseg in range(len(drivebars)-1))
        dpts = [ Along(lam, bar.nodeback.p, bar.nodefore.p)  for bar, lam in drivebars ]
        dptcls = cumlengthlist(dpts)
        ds = Along(alongwire, dptcls[0], dptcls[-1])

        a0, a1, sa = 10, 82, 10
        fout = open("/home/julian/geodesic.csv", "w")
        fout.write("angleout, pathlength, cylangleadvance, anglein\n")
        for i in range((a1 - a0)*sa):
            ldsangle = a0 + i/sa
            gbs1, ds1, dsangle1 = drivegeodesic(drivebars, tridrivebarsmap, dpts, dptcls, ds, ldsangle)
            gbs2 = [ gbs1[-1] ]
            gbs = gbs1
            if ds1 != -1:
                gbs2, ds2, dsangle2 = drivegeodesic(drivebars, tridrivebarsmap, dpts, dptcls, ds1, dsangle1+0)
                gbs = gbs1+gbs2[1:]
            else:
                ds2, dsangle2 = -1, -1
            pathlength = sum((a.pt - b.pt).Len()  for a, b in zip(gbs, gbs[1:]))
            angleadvance = 360*(ds2 - ds)/dptcls[-1] if ds2 != -1 else -1
            fout.write("%f, %f, %f, %f\n" % (ldsangle, pathlength, angleadvance, dsangle2))
            if qoutputfilament.text():
                Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbs1+gbs2[1:]]), qoutputfilament.text())
        fout.close()
                
    else:
        print("Need to select a Sketch and a Mesh object in the UI to make this work")
        qw.hide()


qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 400, 300, 250)
qw.setWindowTitle('Drive geodesic')
qsketchplane = freecadutils.qrow(qw, "Sketchplane: ", 15+35*0)
qmeshobject = freecadutils.qrow(qw, "Meshobject: ", 15+35*1 )
qalongwire = freecadutils.qrow(qw, "Along wire: ", 15+35*2, "0.51")
qanglefilament = freecadutils.qrow(qw, "Angle filament: ", 15+35*3, "30.0")
qoutputfilament = freecadutils.qrow(qw, "Output filament: ", 15+35*4, "w1")
okButton = QtGui.QPushButton("Drive", qw)
okButton.move(180, 15+35*5)
qspraylines = QtGui.QPushButton("Spray lines", qw)
qspraylines.move(20, 15+35*5)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  
QtCore.QObject.connect(qspraylines, QtCore.SIGNAL("pressed()"), spraylines)  


qsketchplane.setText(freecadutils.getlabelofselectedsketch())
qmeshobject.setText(freecadutils.getlabelofselectedmesh())

qw.show()


