# -*- coding: utf-8 -*-

# directional geodesics from embedded curve controlling endpoint

# Embed a curve into a mesh so we can head off in different directions and tell when it is crossed

import Draft, Part, Mesh, MeshPart, Fem
from FreeCAD import Vector, Rotation 
from PySide import QtGui, QtCore

import os, sys, math, time
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])

import freecadutils;  import sys;  sys.modules.pop("freecadutils")

from barmesh.basicgeo import I1, Partition1, P3, P2, Along, lI1
from barmesh.tribarmes.triangleboxing import TriangleBoxing

import freecadutils
freecadutils.init(App)

def TOL_ZERO(X, msg=""):
    if not (abs(X) < 0.0001):
        print("TOL_ZERO fail", X, msg)
        assert False


class BallPathCloseRegions:
    def __init__(self, cp, rad):
        self.cp = cp
        self.rad = rad
        self.radsq = rad*rad
        self.ranges = [ ]

    def addrange(self, rg):
        self.ranges.append(rg)

    def DistPoint(self, p, i):
        if (p - self.cp).Lensq() <= self.radsq:
            self.addrange(I1(i, i))
    
    def DistEdge(self, p0, p1, i):
        v = p1 - p0
        vsq = v.Lensq()
        p0mcp = p0 - self.cp
        lam = -P3.Dot(p0mcp, v)/vsq
        dsq = (p0mcp + v*lam).Lensq()
        if dsq <= self.radsq:
            lamd = math.sqrt((self.radsq - dsq)/vsq)
            TOL_ZERO((p0 + v*(lam + lamd) - self.cp).Len() - self.rad)
            lrg = I1(lam - lamd, lam + lamd)
            if lrg.hi >= 0.0 and lrg.lo <= 1.0:
                self.addrange(I1(i + max(0.0, lrg.lo), i + min(1.0, lrg.hi)))

    def mergeranges(self):
        self.ranges.sort(key=lambda X: X.lo)
        i = 0
        while i < len(self.ranges) - 1:
            if self.ranges[i+1].lo <= self.ranges[i].hi:
                self.ranges[i] = I1(self.ranges[i].lo, max(self.ranges[i].hi, self.ranges[i+1].hi))
                del self.ranges[i+1]
            else:
                i += 1

    def mergegaps(self, d, mandpaths):
        self.ranges.sort(key=lambda X: X.lo)
        i = 0
        while i < len(self.ranges) - 1:
            if self.ranges[i+1].lo > self.ranges[i].hi:
                ld = mandpaths.getgaplength(self.ranges[i].hi, self.ranges[i+1].lo)
                if ld is None or ld > d:
                    i += 1
                    continue
                
            self.ranges[i] = I1(self.ranges[i].lo, max(self.ranges[i].hi, self.ranges[i+1].hi))
            del self.ranges[i+1]


        
class MandrelPaths:
    def __init__(self, mandrelptpaths):
        self.mandrelptpaths = mandrelptpaths
        self.Nm = max(map(len, mandrelptpaths)) + 1
        xrgs = [ ]
        yrgs = [ ]
        for mandrelwindpts in mandrelptpaths:
            xrgs.append(I1.AbsorbList(p.x  for p in mandrelwindpts))
            yrgs.append(I1.AbsorbList(p.y  for p in mandrelwindpts))
        self.xrg = I1.AbsorbList(iter(sum(xrgs, ())))
        self.yrg = I1.AbsorbList(iter(sum(yrgs, ())))

        self.hitreg = [0]*(self.Nm * len(self.mandrelptpaths))
        self.nhitreg = 0

    def encodei(self, j, k):
        return j*self.Nm + k

    def addpathstotgbs(self, tbs):
        for j, mandrelpath in enumerate(self.mandrelptpaths):
            pp = None
            for k, p in enumerate(mandrelpath):
                tbs.AddPoint(p.x, p.y, self.encodei(j, k))
                if pp is not None:
                    i = self.encodei(j, k-1)
                    tbs.AddEdgeR(pp.x, pp.y, p.x, p.y, i)
                    self.getpt(i+1)
                pp = p

    def getpt(self, i):
        mandrelwindpts = self.mandrelptpaths[i // self.Nm]
        k = (i % self.Nm)
        assert k < len(mandrelwindpts), (i, (i // self.Nm), self.Nm, k, len(mandrelwindpts))
        return mandrelwindpts[i % self.Nm]
        
    def getgaplength(self, llam0, llam1):
        si0 = int(llam0)
        si1 = int(llam1)
        ld = 0.0
        if (si0//self.Nm) != (si1//self.Nm):
            return None
        for k in range(si0, si1+1):
            vlen = (self.getpt(k+1) - self.getpt(k)).Len()
            prop = 1.0
            if k == si0 and k == si1:
                prop = llam1 - llam0 
            elif k == si0:
                prop = si0 + 1 - llam0 
            elif k == si1:
                prop = llam1 - si1 
            assert 0.0 <= prop <= 1.0
            ld += prop*vlen
        return ld

def MakeFEAcoloredmesh(mesh, nodecolours):
    tria3 = Fem.FemMesh()
    for p in mesh.Mesh.Points:
        tria3.addNode(p.x, p.y, p.z, p.Index+1)
    for f in mesh.Mesh.Facets:
        tria3.addFace([i+1  for i in f.PointIndices])
    obj = freecadutils.doc.addObject("Fem::FemMeshObject", mesh.Label+"_TH")
    obj.FemMesh = tria3
    obj.Placement = mesh.Placement
    obj.ViewObject.DisplayMode = "Faces, Wireframe & Nodes"
    obj.ViewObject.BackfaceCulling = False
    obj.ViewObject.PointSize = 1
    obj.ViewObject.NodeColor = dict((i+1, col)  for i, col in enumerate(nodecolours))


def okaypressed():
    print("Okay Pressed") 
    mandrelpaths = [ freecadutils.findobjectbylabel(mandrelpathname)  for mandrelpathname in qmandrelpaths.text().split(",") ]
    towwidth = float(qtowwidth.text())
    measuremesh = freecadutils.findobjectbylabel(qmeshpointstomeasure.text())
    col0 = P3(*[float(x.strip())  for x in qcol0.text().split(",") ])
    col1 = P3(*[float(x.strip())  for x in qcol1.text().split(",") ])
    colv0, colv1 = [float(x.strip())  for x in qcolrange.text().split(",") ]
    
    mandrelptpaths = [ ]
    for mandrelpath in mandrelpaths:
        mandrelwindpts = [ P3(p.X, p.Y, p.Z)  for p in mandrelpath.Shape.Vertexes ]
        mandrelptpaths.append(mandrelwindpts)
    mandpaths = MandrelPaths(mandrelptpaths)
    xrg = mandpaths.xrg.Inflate(towwidth*2)
    yrg = mandpaths.yrg.Inflate(towwidth*2)
    boxwidth = max(towwidth, xrg.Leng()/30, yrg.Leng()/30)
    tbs = TriangleBoxing(None, xrg.lo, xrg.hi, yrg.lo, yrg.hi, boxwidth)
    print("Creating box set boxwidth=", boxwidth, mandpaths.Nm)
    mandpaths.addpathstotgbs(tbs)

    thickcount = [ ]
    for mp in measuremesh.Mesh.Points[::]:
        mmp = P3(mp.x, mp.y, mp.z)
        bpcr = BallPathCloseRegions(mmp, towwidth)
        mandpaths.nhitreg += 1
        for ix, iy in tbs.CloseBoxeGenerator(mp.x, mp.x, mp.y, mp.y, towwidth):
            tbox = tbs.boxes[ix][iy]
            for i in tbox.pointis:
                bpcr.DistPoint(mandpaths.getpt(i), i)
            for i in tbox.edgeis:
                if mandpaths.hitreg[i] != mandpaths.nhitreg:
                    bpcr.DistEdge(mandpaths.getpt(i), mandpaths.getpt(i+1), i)
                    mandpaths.hitreg[i] = mandpaths.nhitreg
        bpcr.mergeranges()
        #ss = len(bpcr.ranges)
        #bpcr.mergegaps(0.1, mandpaths)
        #if ss != len(bpcr.ranges):
        #    print("Gap actually merged")
        thickcount.append(len(bpcr.ranges))

    print(col0, col1, colv0, colv1)
    nodecolours = [ ]
    for c in thickcount:
        l = (c - colv0)/(colv1 - colv0)
        nodecolours.append(tuple(Along(max(0, min(1, l)), col0, col1)))
    MakeFEAcoloredmesh(measuremesh, nodecolours)
    qw.hide()

qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 500, 300, 350)
qw.setWindowTitle('Measure thickness')
qmeshpointstomeasure = freecadutils.qrow(qw, "Mesh: ", 15+35*1)

qmandrelpaths = freecadutils.qrow(qw, "Winding paths ", 15+35*2, "")
qtowwidth = freecadutils.qrow(qw, "Tow width: ", 15+35*3, "6")
qcol0 = freecadutils.qrow(qw, "Colour0: ", 15+35*4, "0,0,1")
qcol1 = freecadutils.qrow(qw, "Colour1: ", 15+35*5, "1,0,0")
qcolrange = freecadutils.qrow(qw, "Colrange: ", 15+35*6, "0,5")

okButton = QtGui.QPushButton("Measure", qw)
okButton.move(180, 15+35*7)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  

qmandrelpaths.setText(freecadutils.getlabelofselectedwire(multiples=True))
qmeshpointstomeasure.setText(freecadutils.getlabelofselectedmesh())


qw.show()


#obj.ViewObject.HighlightedNodes = [1, 2, 3]
#The individual elements of a mesh can be modified by passing a dictionary with the appropriate key:value pairs.
#Set volume 1 to red
#obj.ViewObject.ElementColor = {1:(1,0,0)}
#Set nodes 1, 2 and 3 to a certain color; the faces between the nodes acquire an interpolated color.
#obj.ViewObject.NodeColor = {1:(1,0,0), 2:(0,1,0), 3:(0,0,1)}
#Displace the nodes 1 and 2 by the magnitude and direction defined by a vector.
#obj.ViewObject.NodeDisplacement = {1:FreeCAD.Vector(0,1,0), 2:FreeCAD.Vector(1,0,0)}
