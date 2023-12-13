# -*- coding: utf-8 -*-

# stock viewer task, a standard interactive panel on the left in FreeCAD.  
# To edit the UI file, nix-shell -p qtcreator, start up qtcreator
# select default current project and start editing stockviewertask.ui


import Draft, Part, Mesh, MeshPart, Fem
from FreeCAD import Vector, Rotation 
from PySide import QtGui, QtCore


import os, sys, math, time, numpy
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])


from barmesh.basicgeo import I1, Partition1, P3, P2, Along, lI1
from barmesh.tribarmes.triangleboxing import TriangleBoxing
from utils.pathutils import BallPathCloseRegions, MandrelPaths, MakeFEAcoloredmesh
import utils.freecadutils as freecadutils
freecadutils.init(App)
from utils.curvesutils import cumlengthlist, seglampos

import imp
import utils.directedgeodesic
imp.reload(utils.directedgeodesic)
from utils.directedgeodesic import directedgeodesic, makebicolouredwire, makedrivecurve, drivegeodesicRI, DriveCurve
from utils.trianglemeshutils import UsefulBoxedTriangleMesh
from utils.geodesicutils import drivegeodesic, InvAlong, GBarT, GBarC, drivecurveintersectionfinder, trilinecrossing, TOL_ZERO

import FreeCADGui as Gui
import PySide.QtGui as QtGui
import PySide.QtCore as QtCore


def sgetlabelofselectedmesh(sel):
    for s in sel:
        if hasattr(s, "Mesh") and isinstance(s.Mesh, Mesh.Mesh):
            return s.Label
        if s.TypeId == "Mesh::Curvature":
            if "ValueAtIndex" in s.PropertiesList:
                return s.Label
            else:
                print("***  Wrong version of FreeCAD, \n\nMeshCurvature object missing ValueAtIndex hack.")
                return "**WrongFCversion"
    return ""

def sgetlabelofselectedsketch(sel):
    for s in sel:
        if hasattr(s, "Module") and s.Module == 'Sketcher':
            return s.Label
    return ""
    
def sfindobjectbylabel(doc, lab):
    objs = [ obj  for obj in doc.findObjects(Label=lab)  if obj.Label == lab ]
    return objs[0] if objs else None


mandrelradius = 110  # fc6 file
LRdirection = 1
appaturepapproachpoint = P3(0,-150,0)
maxlength = 2500

def findappatureclosestapproach(gbs, sphpt):
    rclose = 0
    iclose = -1
    lamclose = -1.0
    for i in range(1, len(gbs) - 2):
        p0 = gbs[i].pt
        p1 = gbs[i+1].pt
        v0 = p0 - sphpt
        v0len = v0.Len()
        if iclose == -1 or v0len < rclose:
            rclose = v0len
            iclose = i
            lamclose = 0.0
        if v0len > rclose + 10.0:
            continue
        v = p1 - p0
        vlensq = v.Lensq()
        if vlensq != 0:
            lam = -P3.Dot(v0, v) / v.Lensq()
            if 0.0 < lam < 1.0:
                vd = v0 + v*lam
                TOL_ZERO(P3.Dot(vd, v))
                vdlen = vd.Len()
                if vdlen < rclose:
                    rclose = vdlen
                    iclose = i
                    lamclose = lam
    gbt = GBarT([(gbs[iclose].bar, gbs[iclose].lam), (gbs[iclose+1].bar, gbs[iclose+1].lam)], 0, lamclose)
    TOL_ZERO((gbt.pt - sphpt).Len() - rclose)
    return gbt, rclose

def spherecutlam(p0, p1, sphpt, sphrad):
    v = p1 - p0
    vs = p0 - sphpt
    qa = v.Lensq()
    qb2 = P3.Dot(vs, v)
    qc = vs.Lensq() - sphrad*sphrad
    qdq = qb2*qb2 - qa*qc
    qs = math.sqrt(qdq) / qa
    qm = -qb2 / qa
    q = qm + qs
    TOL_ZERO(qa*q*q + qb2*2*q + qc)
    Dpt = vs + v*q
    assert (-0.01 <= q <= 1.01), q
    return q


def TriangleCrossSphereRight(tbar, bGoRight, sphpt, sphrad, bContinuation):
    nodeAhead = tbar.GetNodeFore(bGoRight)
    nodeBehind = tbar.GetNodeFore(not bGoRight)
    barAhead = tbar.GetForeRightBL(bGoRight)
    barAheadGoRight = (barAhead.nodeback == nodeAhead)
    nodeOpposite = barAhead.GetNodeFore(barAheadGoRight)
    barBehind = barAhead.GetForeRightBL(barAheadGoRight)
    DbarBehindGoRight = (barBehind.nodeback == nodeOpposite)
    assert nodeBehind == barBehind.GetNodeFore(DbarBehindGoRight)

    nds = [ nodeBehind, nodeAhead, nodeOpposite ]
    brs = [ tbar, barAhead, barBehind ]
    dnds = [ (b.p - sphpt).Len()  for b in nds ]
    for i in range(3):
        i1 = i + 1 if i != 2 else 0
        if (dnds[i] < sphrad) and (dnds[i1] >= sphrad):
            break
    jlam = spherecutlam(nds[i].p, nds[i1].p, sphpt, sphrad)
    barCrossing = brs[i]
    barCrossingGoRight = not (barCrossing.nodeback == nds[i])
    barCrossingLam = 1 - jlam if barCrossingGoRight else jlam
    barCrossingpt = Along(barCrossingLam, barCrossing.nodeback.p, barCrossing.nodefore.p)
    TOL_ZERO((barCrossingpt - sphpt).Len() - sphrad)
    return barCrossing, barCrossingLam, barCrossingGoRight



def TriangleCrossCutPlane(bar, lam, bGoRight, driveperpvec, driveperpvecDot):
    nodeAhead = bar.GetNodeFore(bGoRight)
    nodeBehind = bar.GetNodeFore(not bGoRight)
    barAhead = bar.GetForeRightBL(bGoRight)
    barAheadGoRight = (barAhead.nodeback == nodeAhead)
    nodeOpposite = barAhead.GetNodeFore(barAheadGoRight)
    barBehind = barAhead.GetForeRightBL(barAheadGoRight)
    DbarBehindGoRight = (barBehind.nodeback == nodeOpposite)
    assert nodeBehind == barBehind.GetNodeFore(DbarBehindGoRight)
    dpvdAhead = P3.Dot(driveperpvec, nodeAhead.p)
    dpvdBehind = P3.Dot(driveperpvec, nodeBehind.p)
    dpvdOpposite = P3.Dot(driveperpvec, nodeOpposite.p)
    assert dpvdAhead > driveperpvecDot - 0.001 and dpvdBehind < driveperpvecDot + 0.001
    bAheadSeg = (dpvdOpposite < driveperpvecDot)
    barCrossing = (barAhead if bAheadSeg else barBehind)
    dpvdAB = (dpvdAhead if bAheadSeg else dpvdBehind)
    barCrossingLamO = -(dpvdOpposite - driveperpvecDot)/(dpvdAB - dpvdOpposite)
    assert barCrossingLamO >= 0.0
    barCrossingLam = barCrossingLamO if (barCrossing.nodeback == nodeOpposite) else 1-barCrossingLamO
    barCrossingGoRight = (barCrossing.nodeback == nodeOpposite) == bAheadSeg
    return barCrossing, barCrossingLam, barCrossingGoRight

def planecutembeddedcurve(startbar, startlam, driveperpvec):
    startpt = Along(startlam, startbar.nodeback.p, startbar.nodefore.p)
    driveperpvecDot = P3.Dot(driveperpvec, startpt)
    drivebars = [ (startbar, startlam) ]
    bGoRight = (P3.Dot(driveperpvec, startbar.nodefore.p - startbar.nodeback.p) > 0)
    bar, lam = startbar, startlam
    while True:
        bar, lam, bGoRight = TriangleCrossCutPlane(bar, lam, bGoRight, driveperpvec, driveperpvecDot)
        drivebars.append((bar, lam))
        if bar == startbar:
            assert abs(startlam - lam) < 0.001
            drivebars[-1] = drivebars[0]
            break
        if len(drivebars) > 500:
            print("failed, too many drivebars")
            break
    return drivebars



def makeappatgurecircuit(gbt):
    startbar, startlam = planecutbars(utbm.tbarmesh, driveperpvec, driveperpvecDot)
    drivebars = planecutembeddedcurve(startbar, startlam, driveperpvec)
    drivecurve = DriveCurve(drivebars)
    print("girth comparison", 'Nominal:',mandrelgirth, 'Drivecurve length:',drivecurve.dptcls[-1])
    return drivecurve

def makesplaycycle(gbs, sphpt):
    gbt, sphrad = findappatureclosestapproach(gbs, sphpt)
    bar, lam, bGoRight = TriangleCrossSphereRight(gbt.tbar, True, sphpt, sphrad, False)
    bar0 = bar
    sphbars = [ GBarC(bar, lam, bGoRight) ]
    for i in range(600):
        bar, lam, bGoRight = TriangleCrossSphereRight(bar, bGoRight, sphpt, sphrad, True)
        sphbars.append(GBarC(bar, lam, bGoRight))
        if bar == bar0:
            break
    return sphbars, sphrad
    
def setpropertyval(obj, atype, name, value):
    if name not in obj.PropertiesList:
        obj.addProperty(atype, name, "filwind")
    setattr(obj, name, value)

class DirectedSplayTaskPanel(QtGui.QWidget):
    def __init__(self):
        x = os.path.join(os.path.split(__file__)[0], "directedsplaytask.ui")
        self.form = FreeCADGui.PySideUic.loadUi(x)
        self.form.setMinimumSize(0, 300)
        QtCore.QObject.connect(self.form.pushButton, QtCore.SIGNAL("pressed()"), self.pushbutton)  
        self.update()

    def update(self):
        self.doc = App.ActiveDocument
        self.sel = App.Gui.Selection.getSelection()
        self.form.qmeshobject.setText(sgetlabelofselectedmesh(self.sel))
        self.form.qsketchplane.setText(sgetlabelofselectedsketch(self.sel))

    def pushbutton(self):
        print("Pushbutton pressed")

    def apply(self):
        print("apply!!")
        sketchplane = sfindobjectbylabel(self.doc, self.form.qsketchplane.text())
        meshobject = sfindobjectbylabel(self.doc, self.form.qmeshobject.text())
        splaycircles = self.form.qsplaycircles.text()
        splayhoops = self.form.qsplayhoops.text()
        alongwire = float(self.form.qalongwire.text())
        minangle = float(self.form.qminangle.text())
        maxangle = float(self.form.qmaxangle.text())
        anglestep = float(self.form.qanglestep.text())

        splaygroup = freecadutils.getemptyfolder(self.doc, splayhoops)
        splaycyclegroup = freecadutils.getemptyfolder(self.doc, splaycircles)
        setpropertyval(splaycyclegroup, "App::PropertyFloat", "alongwire", alongwire)
        setpropertyval(splaycyclegroup, "App::PropertyString", "sketchplane", sketchplane.Label)
        setpropertyval(splaycyclegroup, "App::PropertyString", "meshobject", meshobject.Label)

        utbm = UsefulBoxedTriangleMesh(meshobject.Mesh)
        drivecurve = makedrivecurve(sketchplane, utbm, mandrelradius)

        dsanglesgen = ( minangle + i*anglestep  for i in range(int((maxangle - minangle)/anglestep + 1))  if minangle + i*anglestep < maxangle )
        for dsangle in dsanglesgen:
            gbStart = drivecurve.startalongangle(alongwire, dsangle)
            gbs = drivegeodesicRI(gbStart, drivecurve.drivebars, drivecurve.tridrivebarsmap, LRdirection=LRdirection, sideslipturningfactor=0, maxlength=maxlength)
            if gbs[-1] == None:
                continue
            alongwirelanded, angcrosslanded = drivecurve.endalongpositionA(gbs[-1])
            cgbs, sphrad = makesplaycycle(gbs, appaturepapproachpoint)
            
            name = 'w%.1f' % dsangle

            ply = Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in gbs]), name)
            splaygroup.addObject(ply)
            setpropertyval(ply, "App::PropertyFloat", "alongwire", alongwire)
            setpropertyval(ply, "App::PropertyAngle", "dsangle", dsangle)
            setpropertyval(ply, "App::PropertyFloat", "alongwirelanded", alongwirelanded)
            setpropertyval(ply, "App::PropertyAngle", "angcrosslanded", angcrosslanded)

            cply = Part.show(Part.makePolygon([Vector(*gb.pt)  for gb in cgbs]), name)
            setpropertyval(cply, "App::PropertyFloat", "alongwire", alongwire)
            setpropertyval(cply, "App::PropertyAngle", "dsangle", dsangle)
            setpropertyval(cply, "App::PropertyFloat", "alongwirelanded", alongwirelanded)
            setpropertyval(cply, "App::PropertyAngle", "angcrosslanded", angcrosslanded)
            setpropertyval(cply, "App::PropertyFloat", "sphrad", sphrad)
            splaycyclegroup.addObject(cply)


    def getStandardButtons(self):
        return int(QtGui.QDialogButtonBox.Cancel
                   | QtGui.QDialogButtonBox.Ok
                   | QtGui.QDialogButtonBox.Apply)

    def clicked(self, bt):
        if bt == QtGui.QDialogButtonBox.Apply:
            self.apply()

    def accept(self):
        print("Accept")
        self.apply()
        self.finish()

    def reject(self):
        print("Reject")
        self.finish()

    def finish(self):
        print("Finish")
        Gui.Control.closeDialog()

Gui.Control.showDialog(DirectedSplayTaskPanel())
