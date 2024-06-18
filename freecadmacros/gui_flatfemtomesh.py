# -*- coding: utf-8 -*-
# Use the MeshPart library to project a selected wire onto a selected mesh

import Draft, Part, Mesh, MeshPart, Fem
from FreeCAD import Vector, Rotation 

from PySide import QtGui, QtCore

import os, sys, math, time
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])

#import utils.trianglemeshutils;  import sys;  sys.modules.pop("trianglemeshutils")
import utils.freecadutils as freecadutils 
from utils.trianglemeshutils import UsefulBoxedTriangleMesh
from barmesh.basicgeo import I1, Partition1, P3, P2, Along

freecadutils.init(App)


def femmeshtofacets(fm):
    nodes = fm.Nodes
    facets = [ ]
    for i in fm.FacesOnly:
        t = fm.getElementNodes(i)
        facet = [ fm.getNodeById(t[0]), fm.getNodeById(t[1]), fm.getNodeById(t[2]) ]
        facets.append(facet)
    return facets


def femmeshtofacetsControlPoints(fm):
    facets = [ ]
    for i in fm.FacesOnly:
        t = fm.getElementNodes(i)
        cfacets = [ [ fm.getNodeById(t[0]), fm.getNodeById(t[3]), fm.getNodeById(t[5]) ],
                    [ fm.getNodeById(t[5]), fm.getNodeById(t[3]), fm.getNodeById(t[4]) ],
                    [ fm.getNodeById(t[3]), fm.getNodeById(t[1]), fm.getNodeById(t[4]) ],
                    [ fm.getNodeById(t[5]), fm.getNodeById(t[4]), fm.getNodeById(t[2]) ] ]
        facets.extend(cfacets)
    return facets

def convertmidpointstocontrolpoints(tria6):
    c3 = tria6[3]*2 - tria6[0]*0.5 - tria6[1]*0.5 
    c4 = tria6[4]*2 - tria6[1]*0.5 - tria6[2]*0.5 
    c5 = tria6[5]*2 - tria6[2]*0.5 - tria6[0]*0.5 
    return (tria6[0], tria6[1], tria6[2], c3, c4, c5)

def femmeshtofacetsSubdiv(fm, nrows):
    facets = [ ]
    for i in fm.FacesOnly:
        tria6 = [ fm.getNodeById(j)  for j in fm.getElementNodes(i) ]
        tria6 = convertmidpointstocontrolpoints(tria6)
        prevrow = [ ]
        tfacets = [ ]
        for r in range(nrows + 1):
            s = 1 - r/nrows
            row = [ ]
            for m in range(r + 1):
                t = (m/r if r != 0 else 0)*(1-s)
                u = 1 - s - t
                row.append(evaltria6(tria6, s, t, u))
            for a in range(len(prevrow)):
                tfacets.append([row[a], prevrow[a], row[a+1]])
                if a + 1 < len(prevrow):
                    tfacets.append([prevrow[a], prevrow[a+1], row[a+1]])
            prevrow = row
        facets.extend(tfacets)
    return facets

bflateval = False
def evaltria6(tria6, s, t, u):
    if bflateval:
        return tria6[0]*s + tria6[1]*t + tria6[2]*u
        
    return tria6[0]*(s*s) + tria6[1]*(t*t) + tria6[2]*(u*u) + \
           tria6[4]*(2*t*u) + tria6[5]*(2*u*s) + tria6[3]*(2*s*t)


def okaypressed():
    global bflateval
    print("Okay Pressed") 
    femmeshobject = freecadutils.findobjectbylabel(qfemmeshobject.text())
    nsubdivs = int(qsubdivs.text())
    bflateval = qoptionflateval.isChecked()
    print("bflateval ", bflateval)
    if femmeshobject and hasattr(s, "FemMesh"):
        if nsubdivs == -1:
            facets = femmeshtofacetsControlPoints(femmeshobject.FemMesh)
        elif nsubdivs == 0:
            facets = femmeshtofacetsSubdiv(femmeshobject.FemMesh, nsubdivs+1)
            #facets = femm-1eshtofacets(femmeshobject.FemMesh)
        else:
            facets = femmeshtofacetsSubdiv(femmeshobject.FemMesh, nsubdivs+1)
            
        mesh = Mesh.Mesh(facets)
        newmesh = freecadutils.findobjectbylabel(qnewmeshobject.text())
        if newmesh == None:
            newmesh = freecadutils.doc.addObject("Mesh::Feature", qnewmeshobject.text())
        newmesh.Mesh = Mesh.Mesh(facets)
        newmesh.ViewObject.Lighting = "Two side"
        newmesh.ViewObject.DisplayMode = "Flat Lines"
        
    else:
        print("Need to select a FEMMesh object in the UI to make this work")
    qw.hide()


qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 400, 300, 200)
qw.setWindowTitle('Flat FEM to mesh')
qfemmeshobject = freecadutils.qrow(qw, "FEM Mesh: ", 50)
qsubdivs = freecadutils.qrow(qw, "subdivs: ", 85, "-1")
#qflatangletolerance = freecadutils.qrow(qw, "flat ang tol: ", 85, "1.0")
qnewmeshobject = freecadutils.qrow(qw, "New Mesh: ", 120)

qoptionflateval = QtGui.QCheckBox("Flat Eval", qw)
qoptionflateval.setChecked(False)
qoptionflateval.move(50, 160)

okButton = QtGui.QPushButton("Convert mesh", qw)
okButton.move(160, 170)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  


sel = App.Gui.Selection.getSelection()
femmeshname = ""
for s in sel:
    if hasattr(s, "FemMesh") and isinstance(s.FemMesh, Fem.FemMesh):
        femmeshname = s.Label
qfemmeshobject.setText(femmeshname)
qnewmeshobject.setText(femmeshname+"_tmesh")
qw.show()


    


