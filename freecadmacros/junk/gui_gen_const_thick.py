# -*- coding: utf-8 -*-

# GENERATE A LAYER OF CONSTANT THICKNESS
# First work out what angle is needed to create a given Polar Opening
# Then this is repeated 100 times to find the average thickness around the Polar Opening and so calculate how many repeats are required.


from PySide import QtGui, QtCore
from FreeCAD import Vector
import Mesh, MeshPart
import numpy as np
import os, sys, math
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])

import imp
import utils.directedgeodesic
imp.reload(utils.directedgeodesic)

import utils.freecadutils as freecadutils
from utils.directedgeodesic import directedgeodesic, makebicolouredwire
from barmesh.basicgeo import I1, Partition1, P3, P2, Along, lI1
from barmesh.tribarmes.triangleboxing import TriangleBoxing
from utils.pathutils import BallPathCloseRegions, MandrelPaths
from utils.curvesutils import thinptstotolerance
from utils.trianglemeshutils import UsefulBoxedTriangleMesh

doc, sel = freecadutils.init(App)
sketchplane = None
meshobject = None
thintol = 0.2

def TOL_ZERO(X, msg=""):
    if not (abs(X) < 0.0001):
        print("TOL_ZERO fail", X, msg)


#Function that repeats a path and evaluates the thickness on a ring at coord yo and radius r
def evalrepthick(landed, totrpt, r, yo, dsangle, tw):
	print('Tot rep', totrpt)
	print('LANDED AT', landed)
	#Check that the pattern won't repeat until it reaches the end
	turns = round(totrpt * (landed))
	if not turns: turns = 1
	for i in range(1,-turns):
		if not totrpt%(turns*1):
			print('WARNING: Ideal number or repeats to build required thickness would produce gappy winding pattern. Adding 1 repeat')
			totrpt+=1
	alongwireadv =  turns / totrpt
	print('FORCE TO', alongwireadv)
	alongwireI = alongwire + alongwireadv
	gbs, fLRdirection, dseg, alongwirelanded100 = directedgeodesic(combofoldbackmode, sketchplane, meshobject, alongwire, alongwireI, dsangle, Maxsideslipturningfactor, mandrelradius, 0.0, maxlength, outputfilament)
	#name = 'w'+str(int(dsangle-90))+'forced'
	#makebicolouredwire(gbs, name, colfront=(1.0,0.0,0.0) if fLRdirection == -1 else (0.0,0.0,1.0), colback=(0.7,0.7,0.0), leadcolornodes=dseg+1)
	rpts = repeatwindingpath([P3(*gb.pt)  for gb in gbs], totrpt,thintol)
	meanthick = evalthick(r, yo, [rpts],tw,tth)
	return rpts, meanthick,totrpt
	
def repeatwindingpath(rpts, repeats,thintol):
	ptfront, ptback = rpts[0], rpts[-1]
	fvec0 = P2(ptfront.x, ptfront.z)
	fvec1 = P2(ptback.x, ptback.z)
	angadvance = P2(P2.Dot(fvec0, fvec1), P2.Dot(fvec0, P2.APerp(fvec1))).Arg()
	rpts = thinptstotolerance(rpts, tol=thintol*2)
	ptsout = rpts[:]
	for i in range(1, repeats):
		rotcos = math.cos(math.radians(i*angadvance))
		rotsin = math.sin(math.radians(i*angadvance))
		for pt in rpts:
			ptsout.append(P3(pt.x*rotcos + pt.z*rotsin, pt.y, pt.z*rotcos - pt.x*rotsin))
	ptsout = thinptstotolerance(ptsout, tol=thintol)
	return ptsout

def evalthick(r, yo, tpt, towwidth, towthick, evalpts=50, tnormPolar=None):
	towwidth /= 2
	POpts=[]
	POnorms = [ ]
	for a in np.linspace(0,2*np.pi,evalpts):
		POpts.append(App.Vector(r*np.sin(a),yo,r*np.cos(a)))
		if tnormPolar:
			POnorms.append(App.Vector(0,1,0))
		
	mandpaths = MandrelPaths(tpt)
	xrg = mandpaths.xrg.Inflate(towwidth*2)
	yrg = mandpaths.yrg.Inflate(towwidth*2)
	boxwidth = max(towwidth, xrg.Leng()/30, yrg.Leng()/30)
	tbs = TriangleBoxing(None, xrg.lo, xrg.hi, yrg.lo, yrg.hi, boxwidth)
	print("Creating box set boxwidth=", boxwidth, mandpaths.Nm)
	mandpaths.addpathstotgbs(tbs)

	thickcount = [ ]
	maxthickcount = 0
	thickpoint = None
	for mp in POpts:
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
		thickcount.append(len(bpcr.ranges))
		if thickcount[-1] > maxthickcount:
			maxthickcount = thickcount[-1]
			thickpoint = mp
	meanthick = np.mean(thickcount)*towthick
	print('THICKNESSES AROUND RING:')
	print('MAX:',np.max(thickcount)*towthick,'MIN:',np.min(thickcount)*towthick,'MEAN:', meanthick)

	if tnormPolar:
		facets = [ ]
		Dexaggeratedtowthick = 1.0
		for i in range(len(POpts) - 1):
			p0, p1 = POpts[i], POpts[i+1]
			pe0, pe1 = p0 + POnorms[i]*(thickcount[i]*Dexaggeratedtowthick), p1 + POnorms[i+1]*(thickcount[i+1]*Dexaggeratedtowthick)
			facets.append([p0, pe0, p1])
			facets.append([p1, pe0, pe1])
		mesh = freecadutils.doc.addObject("Mesh::Feature", "thicknessflange")
		mesh.ViewObject.Lighting = "Two side"
		mesh.ViewObject.DisplayMode = "Flat Lines"
		mesh.Mesh = Mesh.Mesh(facets)

	return meanthick

def drivepressed():
	aimpressed()
	preppressed()
	actpressed()

# XZMin should be same as PolarOpening after aim
# alongwire???
# PolarOpening should be along a perp drive curve?
# Generate a little sector of stock to see how it differs from 6mm (colour it?)

# (this could approach a point instead of the axis
def CalcClosestPolarEdge(gbs):
	ilclosest = 0.0
	dclosestsq = P2(gbs[0].pt.x, gbs[0].pt.z).Lensq()
	tnorm = gbs[0].tnorm  # GBarT kind.  Rest are GBarC kind so have tnorm_incoming
	passY = gbs[0].pt.y
	for i in range(0, len(gbs) - 2):
		p0 = P2(gbs[i].pt.x, gbs[i].pt.z)
		p1 = P2(gbs[i+1].pt.x, gbs[i+1].pt.z)
		ldclosestsq = p1.Lensq()
		if ldclosestsq < dclosestsq:
			dclosestsq = ldclosestsq
			ilclosest = i + 1.0
			tnorm = gbs[i+1].tnorm_incoming
			passY = gbs[i+1].pt.y
		v = p1 - p0
		vsq = v.Lensq()
		if vsq != 0.0:
			lam = -P2.Dot(p0, v) / vsq
			if 0 < lam < 1:
				ldclosestsq = Along(lam, p0, p1).Lensq()
				if ldclosestsq < dclosestsq:
					dclosestsq = ldclosestsq
					ilclosest = i + lam
					Dv = gbs[i+1].pt - gbs[i].pt
					tnorm = gbs[i+1].tnorm_incoming
					TOL_ZERO(P3.Dot(Dv, tnorm))
					passY = Along(lam, gbs[i].pt, gbs[i+1].pt)
	return math.sqrt(dclosestsq), passY, tnorm

tnormattargetPO = P3(0,0,0)
def aimpressed():
	global sketchplane, outputfilament, thintol, tnormattargetPO
	dsangle = None
	sketchplane = freecadutils.findobjectbylabel(qsketchplane.text())
	meshobject = freecadutils.findobjectbylabel(qmeshobject.text())
	outputfilament = qoutputfilament.text()
	targetPO = float(qtargetPO.text())
	totthick = float(qtotthick.text())
	tolPO = float(qtolPO.text())
	AngLo = 90+float(qAngLo.text())
	AngHi = 90+float(qAngHi.text())
	thintol = float(qthintol.text())
	tth = float(qtowthick.text())
	
	alongwireI = None
	finished = False
	attempts = 10
	i = 0
	utbm = UsefulBoxedTriangleMesh(meshobject.Mesh)
	while not finished:
		dsangle = (AngHi+AngLo)/2
		print('Trying angle: ', dsangle-90)
		gbs, fLRdirection, dseg, alongwirelanded = directedgeodesic(combofoldbackmode, sketchplane, utbm, alongwire, alongwireI, dsangle, Maxsideslipturningfactor, mandrelradius, 0.0, maxlength, outputfilament)
		XZmin, passY, tnorm = CalcClosestPolarEdge(gbs)
		print('  New Closest approach to Y axis of:',round(XZmin,2), 'at:', passY.x,',', passY.y,',',passY.z)
		if XZmin < targetPO or gbs[-1] == None:
			AngLo = dsangle
		else:
			AngHi = dsangle
		if i > attempts:
			print('**** WARNING *****')
			print('Failed to find path to satisfy')
			finished = True
		elif abs(XZmin-targetPO) < tolPO and gbs[-1]:
			print('Angle of',dsangle-90, 'deg found to give Polar Opening of', XZmin, 'at Y of', passY.y)
			finished = True
		i+=1
	qalongwire.setText("%.6f" % alongwire)
	qdsangle.setText("%.6f" % dsangle)
	qXZmin.setText("%.6f" % XZmin)
	qpassYy.setText("%.6f" % passY.y)
	qalongwirelanded.setText("%.6f" % alongwirelanded)
	tnormattargetPO = tnorm
	
def preppressed():
	global sketchplane, meshobject, outputfilament , thintol
	dsangle = None
	sketchplane = freecadutils.findobjectbylabel(qsketchplane.text())
	meshobject = freecadutils.findobjectbylabel(qmeshobject.text())
	outputfilament = qoutputfilament.text()
	totthick = float(qtotthick.text())
	thintol = float(qthintol.text())
	tw = float(qtowwidth.text())
	tth = float(qtowthick.text())
	
	alongwire = float(qalongwire.text())
	alongwirelanded = float(qalongwirelanded.text())
	dsangle = float(qdsangle.text())
	XZmin = float(qXZmin.text())
	passYy = float(qpassYy.text())
	thickgroup = freecadutils.findobjectbylabel(qthickgroup.text()) if qthickgroup.text() != "" else None
	if not (sketchplane and meshobject):
		print("Need to select a Sketch and a Mesh object in the UI to make this work")
		qw.hide()
		return

	# Insert another separation point here.
	# XZmin, dsangle, passYy
	
	name = 'w'+str(int(dsangle-90))
	#makebicolouredwire(gbs, name, colfront=(1.0,0.0,0.0) if fLRdirection == -1 else (0.0,0.0,1.0), colback=(0.7,0.7,0.0), leadcolornodes=dseg+1)
	doc.recompute()
	
	#REPEAT 101 times
	totrpt = 101
	landed = alongwirelanded - alongwire
	rpts, meanthick, totrpt = evalrepthick(landed, totrpt, XZmin, passYy, dsangle, tw)
	#Create wire forced to an intersection point that gives an integer number of repeat)
	basethick = 0.0
	if thickgroup:
		basepts = []
		for bw in thickgroup.OutList:
			if hasattr(bw, "Shape") and isinstance(bw.Shape, Part.Wire) and "towwidth" in bw.PropertiesList:
				print("--evalthick includes:", bw.Name)
				basepts.append([P3(v.X,v.Y,v.Z)  for v in bw.Shape.Vertexes])
		tnormPolar = P2(P2(tnormattargetPO.x, tnormattargetPO.z).Len(), tnormattargetPO.y)
		basethick = evalthick(XZmin, passYy, basepts, tw, tth, tnormPolar=tnormPolar)
		print('basethickness', basethick)
	totrpt = int(totrpt*(totthick-basethick)/meanthick)-1
	
	qtotrpt.setText("%d" % totrpt)
	qlanded.setText("%.6f" % landed)

	
def actpressed():
	global sketchplane, meshobject, outputfilament , thintol
	sketchplane = freecadutils.findobjectbylabel(qsketchplane.text())
	meshobject = freecadutils.findobjectbylabel(qmeshobject.text())
	outputfilament = qoutputfilament.text()
	totthick = float(qtotthick.text())
	thintol = float(qthintol.text())
	tth = float(qtowthick.text())
	tw = float(qtowwidth.text())

	alongwire = float(qalongwire.text())
	dsangle = float(qdsangle.text())
	totrpt = int(qtotrpt.text())
	landed = float(qlanded.text())
	XZmin = float(qXZmin.text())
	passYy = float(qpassYy.text())

	if qthickgroup.text() != "":
		thickgroup = freecadutils.findobjectbylabel(qthickgroup.text())
	else:
		thickgroup = freecadutils.getemptyfolder(doc, "Constant thickness")
		qthickgroup.setText(thickgroup.Name)
		
	#Repeat that wire to create final ply
	rpts, meanthick,totrpt = evalrepthick(landed, totrpt, XZmin, passYy, dsangle, tw)
	print('First:', rpts[0], 'Last', rpts[-1] , 'thick', meanthick)
	name = 'w%dx%d' % (int(dsangle-90), totrpt)
	ply = Part.show(Part.makePolygon([Vector(pt)  for pt in rpts]), name)
	thickgroup.addObject(ply)
	targetPO = float(qtargetPO.text())
	ply.addProperty("App::PropertyAngle", "dsangle", "filwind"); ply.dsangle = dsangle
	ply.addProperty("App::PropertyFloat", "alongwire", "filwind"); ply.alongwire = alongwire
	ply.addProperty("App::PropertyFloat", "totrpt", "filwind"); ply.totrpt = totrpt
	ply.addProperty("App::PropertyFloat", "landed", "filwind"); ply.landed = landed
	ply.addProperty("App::PropertyFloat", "towwidth", "filwind"); ply.towwidth = tw
	ply.addProperty("App::PropertyFloat", "targetPO", "filwind"); ply.targetPO = targetPO

	#Part.show(Part.makePolygon([rpts[0],rpts[-1]]), 'grrr')
	
	# advancing in prep for next cycle
	targetPO = float(qtargetPO.text())
	tw = float(qtowwidth.text())
	qtargetPO.setText("%.6f" % (targetPO+tw*0.75))



Maxsideslipturningfactor = 0.26
combofoldbackmode = 0
mandrelradius = 110  # fc6 file


maxlength = 6000

alongwire = 0.51
tth = 0.18

qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 500, 570, 490)
qw.setWindowTitle('Generate constant thickness wind')
qsketchplane = freecadutils.qrow(qw, "Sketchplane: ", 15+35*0)
qmeshobject = freecadutils.qrow(qw, "Meshobject: ", 15+35*1 )

qAngLo = freecadutils.qrow(qw, "Min angle: ", 15+35*2, "%.1f" % 10)
qAngHi = freecadutils.qrow(qw, "Max angle: ", 15+35*3, "%.1f" % 90)
qoutputfilament = freecadutils.qrow(qw, "Output name: ", 15+35*10, "w1", 260)
qthintol = freecadutils.qrow(qw, "Thinning tol: ", 15+35*5, "0.2")
qalongwirelanded = freecadutils.qrow(qw, "alongwirelanded: ", 15+35*6)
qthickgroup = freecadutils.qrow(qw, "Thick group: ", 15+35*11, "", 260)


qtargetPO = freecadutils.qrow(qw, "Polar opening r: ", 15+35*0, "%.2f" % 43.0, 260)
qtowwidth = freecadutils.qrow(qw, "Tow width: ", 15+35*2, "%.2f" % 6.35, 260)
qtolPO = freecadutils.qrow(qw, "PO tolerance: ", 15+35*1, "%.2f" % (6.35/8), 260)
qtowthick = freecadutils.qrow(qw, "Tow thickness: ", 15+35*3, "%.2f" % tth, 260)
#vlab = QtGui.QLabel("clear above to go one direction", qw)
#vlab.move(20+260, 15+35*3+5)

aimButton = QtGui.QPushButton("Aim", qw)
aimButton.move(90, 15+35*4)
QtCore.QObject.connect(aimButton, QtCore.SIGNAL("pressed()"), aimpressed)  

qtotthick = freecadutils.qrow(qw, "Desired thick: ", 15+35*4, "6.0", 260)

prepButton = QtGui.QPushButton("Prep", qw)
prepButton.move(90, 15+35*9)
QtCore.QObject.connect(prepButton, QtCore.SIGNAL("pressed()"), preppressed)  

qalongwire = freecadutils.qrow(qw, "alongwire: ", 15+35*7, "", 260)
qdsangle = freecadutils.qrow(qw, "dsangle: ", 15+35*7, "")
qtotrpt = freecadutils.qrow(qw, "totrpt: ", 15+35*9, "", 260)
qlanded = freecadutils.qrow(qw, "landed: ", 15+35*10, "")
qXZmin = freecadutils.qrow(qw, "XZmin: ", 15+35*8, "", 260)
qpassYy = freecadutils.qrow(qw, "passYy: ", 15+35*8, "")


actButton = QtGui.QPushButton("Act", qw)
actButton.move(90, 15+35*12)
QtCore.QObject.connect(actButton, QtCore.SIGNAL("pressed()"), actpressed)  

driveButton = QtGui.QPushButton("Drive", qw)
driveButton.move(190, 15+35*12)
QtCore.QObject.connect(driveButton, QtCore.SIGNAL("pressed()"), drivepressed)  


qsketchplane.setText(freecadutils.getlabelofselectedsketch())
qmeshobject.setText(freecadutils.getlabelofselectedmesh())

qw.show()
