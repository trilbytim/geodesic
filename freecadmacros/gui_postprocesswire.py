# -*- coding: utf-8 -*-

# directional geodesics from embedded curve controlling endpoint

# Embed a curve into a mesh so we can head off in different directions and tell when it is crossed

import Draft, Part, Mesh, MeshPart
from FreeCAD import Vector, Rotation 
from PySide import QtGui, QtCore

import os, sys, math, time
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])

import curvesutils;  import sys;  sys.modules.pop("curvesutils")
import trianglemeshutils;  import sys;  sys.modules.pop("trianglemeshutils")
import geodesicutils;  import sys;  sys.modules.pop("geodesicutils")
import freecadutils;  import sys;  sys.modules.pop("freecadutils")

from barmesh.basicgeo import I1, Partition1, P3, P2, Along
from curvesutils import isdiscretizableobject, discretizeobject, thinptstotolerance
from curvesutils import cumlengthlist, seglampos
from trianglemeshutils import UsefulBoxedTriangleMesh, facetbetweenbars
from wireembeddingutils import planecutembeddedcurve, planecutbars

from geodesicutils import drivegeodesic, InvAlong, GBarT, GBarC, drivecurveintersectionfinder, trilinecrossing, TOL_ZERO

import freecadutils
freecadutils.init(App)

    
    
# X - moves towards/away from the tool (perpendicular to the carriage).
# Y - moves along the carriage axis
# Z - Vertical positioning (rarely used)
# A - Yaw of the robot (around the Z axis)
# E1 - Rotation of the payout eye
# E2 - Angle we roll out from the payout eye (not a real control)
# E3 - Rotation both chucks (headstock and tailstock)
# E4 - Rotation of tailstock relative to headstock (rarely used)

def outputsrclines(pts, freetapelength, robotstate):
    prevE1 = robotstate["E1"]  # used to track the number of complete winds on the eyelet part
    prevE3 = robotstate["E3"]  # used to track the number of complete winds on the lathe part
    res = [ ]

    # loop through the points and vectors 
    # (we use the incoming vector at each triangle edge, not the outgoing vector)
    for i in range(1, len(pts)):
        pt = pts[i]
        vec = P3.ZNorm(pts[i] - pts[i-1])
        tapevector = vec*freetapelength
        tcp = pt + tapevector
        radialvec = P2(tcp.x, tcp.z)
        lE3 = radialvec.Arg()
        E3 = lE3 + 360*int((abs(prevE3 - lE3)+180)/360)*(1 if prevE3 > lE3 else -1)
        X = -radialvec.Len()
        tangentunitvec = -P2.CPerp(radialvec)*(1/X)
        E1vec = P2(-P2.Dot(tangentunitvec, P2(tapevector.x, tapevector.z)), -tapevector.y)
        lE1 = E1vec.Arg()
        E1 = lE1 + 360*int((abs(prevE1 - lE1)+180)/360)*(1 if prevE1 > lE1 else -1)
        res.append({"X":X, "Y":tcp.y, "E1":E1, "E3":E3})
        prevE1 = E1
        prevE3 = E3
        robotstate["Y"] = tcp.y
        robotstate["X"] = X
    robotstate["E1"] = prevE1
    robotstate["E3"] = prevE3

    return res

def srcpt(ps):
	return "{%s}" % ", ".join("%s %+9.3f" % (c, ps[c])  for c in ["X", "Y", "Z", "A", "E1", "E3", "E4"]  if c in ps)



def projectToXvalplane(pt, vec, cx):
    qa = P2(vec.x, vec.z).Lensq()
    qb2 = pt.x*vec.x + pt.z*vec.z
    qc = P2(pt.x, pt.z).Lensq() - cx*cx
    qdq = qb2*qb2 - qa*qc
    qs = math.sqrt(qdq) / qa
    qm = -qb2 / qa
    q = qm + qs
    TOL_ZERO(qa*q*q + qb2*2*q + qc)
    res = pt + vec*q
    TOL_ZERO(P2(res.x, res.z).Len() - cx)
    return res

class TCPplusfibre:
    def __init__(self, tcpR, ptR, prevtcp):
        vecR = ptR - tcpR
        self.freefibrelength = vecR.Len()
        self.DvecR = vecR
        
        lE3 = P2(-tcpR.x, -tcpR.z).Arg()
        prevE3 = prevtcp.E3 if prevtcp is not None else lE3
        self.E3 = lE3 + 360*int((abs(prevE3 - lE3)+180)/360)*(1 if prevE3 > lE3 else -1)
        
        tcpRd = P2(tcpR.x, tcpR.z).Len()
        rvX = P3(-tcpR.x/tcpRd, 0, -tcpR.z/tcpRd)
        rvY = P3(0, 1, 0)
        rvZ = P3(-rvX.z, 0, rvX.x)
        self.X = -tcpRd
        self.Y = tcpR.y
        self.Z = 0
        
        DtcpR = rvX*self.X + rvY*self.Y + rvZ*self.Z
        TOL_ZERO((DtcpR - tcpR).Len())
        vec = P3(P3.Dot(rvX, vecR), P3.Dot(rvY, vecR), P3.Dot(rvZ, vecR))
        TOL_ZERO(self.freefibrelength - vec.Len())
        TOL_ZERO((self.RotByE3(vec) - vecR).Len(), ("rotByE3vec failed"))


        self.E1 = P2(vec.z, vec.y).Arg()
        self.E2 = math.degrees(math.acos(vec.x/self.freefibrelength))
        TOL_ZERO((tcpR - self.GetTCP(True)).Len())
        TOL_ZERO((vecR - self.GetVecR(True)).Len(), (vecR, self.GetVecR(True)))

    def RotByE3(self, pt):
        rvX = P3(math.cos(math.radians(self.E3)), 0, math.sin(math.radians(self.E3)))
        rvZ = P3(-rvX.z, 0, rvX.x)
        return rvX*pt.x + P3(0, pt.y, 0) + rvZ*pt.z
    
    def GetTCP(self, bRotated):
        res = P3(self.X, self.Y, self.Z)
        return self.RotByE3(res) if bRotated else res
        
    def GetVecR(self, bRotated):
        cosE2 = math.cos(math.radians(self.E2))
        sinE2 = math.sin(math.radians(self.E2))
        cosE1 = math.cos(math.radians(self.E1))
        sinE1 = math.sin(math.radians(self.E1))
        res = P3(cosE2, sinE2*sinE1, sinE2*cosE1)*self.freefibrelength
        return self.RotByE3(res) if bRotated else res


def okaypressed():
    print("Okay Pressed") 
    toolpathobject = freecadutils.findobjectbylabel(qtoolpath.text())

    tapecurve = [ P3(p.X, p.Y, p.Z)  for p in toolpathobject.Shape.Vertexes ]
    tcpconstXval = float(qxconst.text())

    textlen = float(qtoolpathlength.text()) if len(qtoolpathlength.text()) != 0 and qtoolpathlength.text()[-1] != " " else None
    
    tcps = [ ]
    for i in range(len(tapecurve)):
        vecNout = P3.ZNorm(tapecurve[max(i,1)] - tapecurve[max(i,1)-1])
        ptR = tapecurve[i]
        tcpR = projectToXvalplane(ptR, vecNout, abs(tcpconstXval))
        prevtcp = tcps[-1] if len(tcps) != 0 else None
        tcp = TCPplusfibre(tcpR, ptR, prevtcp)
        tgcpmov = (prevtcp.GetTCP(True) - tcp.GetTCP(True)).Len() if prevtcp != None else 1000
        if tgcpmov < 0.05:
            print("Skipping trivial aligned tcp motion", i, tgcpmov)
        else:
            tcps.append(tcp)
        if i != 0 and textlen is not None:
            textlen -= (tapecurve[i] - tapecurve[i-1]).Len()
            if textlen <= 0:
                break
    
    foutputsrc = qoutputsrcfile.text()
    headersrc = os.path.join(os.path.split(__file__)[0], "header.src")
    print("outputting src toolpath ", os.path.abspath(foutputsrc))

    tcpblocks = [ [ tcps[0] ] ]
    for itcp in range(1, len(tcps)):
        tcp = tcps[itcp]
        tcpprev = tcpblocks[-1][-1]
        Ydirectionchange = (len(tcpblocks[-1]) >= 2 and ((tcpblocks[-1][-2].Y < tcpblocks[-1][-1].Y) != (tcpblocks[-1][-1].Y < tcp.Y)))
        Yhardswitchback = P3.Dot(P3.ZNorm(tcpprev.GetVecR(True)), P3.ZNorm(tcp.GetVecR(True))) < -0.5
        if Yhardswitchback:
            print("Yhardswitchback", itcp, Ydirectionchange)
            assert Ydirectionchange, "hardswitchback should coincide with a Ydirection change"
        if Ydirectionchange:
            tcpblocks.append([ tcpprev ])
        tcpblocks[-1].append(tcp)
            
    if len(qoutputsweepmesh.text()) != 0 and qoutputsweepmesh.text()[-1] != "*":
        facets = [ ]
        for i in range(itcp-1):
            tcp0, tcp1 = tcps[i].GetTCP(True), tcps[i+1].GetTCP(True)
            vecr0, vecr1 = tcps[i].GetVecR(True), tcps[i+1].GetVecR(True)
            fp0, fp1 = tcp0 + vecr0, tcp1 + vecr1
            if (fp0 - P3(-108.77,750,61.57)).Len() > 6:  continue
            facets.append([Vector(*tcp0), Vector(*fp0), Vector(*tcp1)])
            facets.append([Vector(*tcp1), Vector(*fp0), Vector(*fp1)])
        mesh = freecadutils.doc.addObject("Mesh::Feature", qoutputsweepmesh.text())
        mesh.ViewObject.Lighting = "Two side"
        mesh.ViewObject.DisplayMode = "Flat Lines"
        mesh.Mesh = Mesh.Mesh(facets)

    print("blocks ", list(map(len, tcpblocks)))

    def srcpt(ps):
        return "{%s}" % ", ".join("%s %+9.3f" % (c, ps[c])  for c in ["X", "Y", "Z", "A", "E1", "E3", "E4"]  if c in ps)

    Ymid = 1000
    def srctcp(tcp):
        return srcpt({"X":tcp.X, "Y":tcp.Y + Ymid, "E1":tcp.E1, "E3":tcp.E3*1000/360})

    headersrc = os.path.join(os.path.split(__file__)[0], "header.src")
    print("making toolpath: ", os.path.abspath(foutputsrc))
    fout = open(foutputsrc, "w")
    fout.write(open(headersrc).read())
    fout.write("SLIN %s\n" % srcpt({"X":-200, "Y":Ymid, "Z":0, "A":0, "E1":0, "E3":0, "E4":0}))
    for i in range(len(tcpblocks)):
        tcpblock = tcpblocks[i]
        if i == 0:
            fout.write("\nSLIN %s\n" % srctcp(tcpblock[0]))
            fout.write("HALT\n")
        else:
            Yhardswitchback = P3.Dot(P3.ZNorm(tcpblock[0].GetVecR(True)), P3.ZNorm(tcpblock[1].GetVecR(True))) < -0.5
            if Yhardswitchback:
                fout.write("HALT  ; switchback \n")
                fout.write("SLIN %s\n\n" % srctcp(tcpblock[1]))
        fout.write("SPLINE\n")
        for tcp in tcpblock[2 if Yhardswitchback else 1:]:
            fout.write("SPL %s\n" % srctcp(tcp))
        fout.write("ENDSPLINE\n\n")
    fout.write("SLIN %s\n" % srcpt({"X":-200, "Y":Ymid, "E1":0}))
    fout.write("HALT\nEND\n")
    fout.close()

    
qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 500, 570, 350)
qw.setWindowTitle('Post process toolpath')
qtoolpath = freecadutils.qrow(qw, "Toolpath: ", 15+35*1)
qoutputsrcfile = freecadutils.qrow(qw, "Output file: ", 15+35*4, os.path.abspath("filwin10.src"))
qthintol = freecadutils.qrow(qw, "Thinning tol: ", 15+35*5, "0.2")
qoutputsweepmesh = freecadutils.qrow(qw, "sweepmesh: ", 15+35*6, "m1*")

okButton = QtGui.QPushButton("Post", qw)
okButton.move(180, 15+35*7)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  

qtoolpath.setText(freecadutils.getlabelofselectedwire())
qxconst = freecadutils.qrow(qw, "xconst: ", 15+35*2, "-115")
qtoolpathlength = freecadutils.qrow(qw, "(Length): ", 15+35*1, "0 ", 260)

toolpathobject = freecadutils.findobjectbylabel(qtoolpath.text())
if toolpathobject:
    qtoolpathlength.setText("%.0f " % toolpathobject.Shape.Length)
    qxconst.setText("%.1f" % (toolpathobject.Shape.BoundBox.XMin - 5.0))

qw.show()


