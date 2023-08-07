# -*- coding: utf-8 -*-

# GENERATE A LAYER OF CONSTANT THICKNESS
# First work out what angle is needed to create a given Polar Opening
# Then this is repeated 100 times to find the average thickness around the Polar Opening and so calculate how many repeats are required.


from PySide import QtGui, QtCore
from FreeCAD import Vector
import numpy as np
import os, sys, math
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])

import utils.freecadutils as freecadutils
from utils.directedgeodesic import directedgeodesic, makebicolouredwire
from barmesh.basicgeo import I1, Partition1, P3, P2, Along, lI1
from barmesh.tribarmes.triangleboxing import TriangleBoxing
from utils.pathutils import BallPathCloseRegions, MandrelPaths

doc, sel = freecadutils.init(App)

#def repeatwindingpath(Ssinglewindpts, repeats):
#	ptfront, ptback = Ssinglewindpts[0][0], Ssinglewindpts[-1][-1]
#	fvec0 = P2(ptfront.x, ptfront.z)
#	fvec1 = P2(ptback.x, ptback.z)
#	angadvance = P2(P2.Dot(fvec0, fvec1), P2.Dot(fvec0, P2.APerp(fvec1))).Arg()
#	tpt0 = Ssinglewindpts[0][:]
#	#tsinglewindpts = thinptstotolerance(singlewindpts, tol=thintol)
#	for singlewindpts in Ssinglewindpts[1:]:
#		tpt0.extend(singlewindpts[1:])
 #       
#	tpt =  [P3(p.x, p.y, p.z)  for p in tpt0[:]]
#	for i in range(1, repeats):
#		rotcos = math.cos(math.radians(i*angadvance))
#		rotsin = math.sin(math.radians(i*angadvance))
#		for pt in tpt0[1:]:
#			tpt.append(P3(pt.x*rotcos + pt.z*rotsin, pt.y, pt.z*rotcos - pt.x*rotsin))
#	return tpt
	
def repeatwindingpath(rpts, repeats):
	ptfront, ptback = rpts[0], rpts[-1]
	fvec0 = P2(ptfront.x, ptfront.z)
	fvec1 = P2(ptback.x, ptback.z)
	angadvance = P2(P2.Dot(fvec0, fvec1), P2.Dot(fvec0, P2.APerp(fvec1))).Arg()
	#tsinglewindpts = thinptstotolerance(singlewindpts, tol=thintol)
        
	for i in range(1, repeats):
		rotcos = math.cos(math.radians(i*angadvance))
		rotsin = math.sin(math.radians(i*angadvance))
		for pt in rpts:
			rpts.append(P3(pt.x*rotcos + pt.z*rotsin, pt.y, pt.z*rotcos - pt.x*rotsin))
	return rpts

def evalthick(r, yo, tpt, towwidth, towthick):
	POpts=[]
	for a in np.linspace(0,2*np.pi,20):
		POpts.append(App.Vector(r*np.sin(a),yo,r*np.cos(a)))
	
	mandpaths = MandrelPaths(tpt)
	xrg = mandpaths.xrg.Inflate(towwidth)
	yrg = mandpaths.yrg.Inflate(towwidth)
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
	return meanthick

def okaypressed():
	sketchplane = freecadutils.findobjectbylabel(qsketchplane.text())
	meshobject = freecadutils.findobjectbylabel(qmeshobject.text())
	outputfilament = qoutputfilament.text()
	targetPO = float(qtargetPO.text())
	totthick = float(qtotthick.text())
	tolPO = float(qtolPO.text())
	AngLo = float(qAngLo.text())
	AngHi = float(qAngHi.text())
	if not (sketchplane and meshobject):
		print("Need to select a Sketch and a Mesh object in the UI to make this work")
		qw.hide()
		return
	
	sideslipturningfactorZ = float(qsideslip.text())
	
#	if len(qalongwireadvanceI.text()) != 0:
#		alongwireadvanceI = float(qalongwireadvanceI.text())
#		alongwireI = (alongwire + alongwireadvanceI) % 1.0
#	else:
#		alongwireI = None
	alongwireI = None
	finished = False
	attempts = 5
	i = 0
	while not finished:
		dsangle = 90-(AngHi+AngLo)/2
		print('Trying angle: ', 90 - dsangle)
		gbs, fLRdirection, dseg, alongwirelanded = directedgeodesic(combofoldbackmode, sketchplane, meshobject, alongwire, alongwireI, dsangle, Maxsideslipturningfactor, mandrelradius, sideslipturningfactorZ, maxlength, outputfilament)
		if gbs[-1]:
			pts = [Vector(*gb.pt)  for gb in gbs]
		else:
			pts = [Vector(*gb.pt)  for gb in gbs[:-1]]
		#(could do with better way of seeing closest approach to y axis)
		XZmin = np.Inf
		passY = None
		for v in pts:
			XZ = np.sqrt(v.x**2 + v.z**2)
			if XZ < XZmin:
				XZmin = XZ
				passY = v
		print('Closest approach to Y axis of:',round(XZmin,2), 'at:', passY.x,',', passY.y,',',passY.z)
		if XZmin < targetPO or gbs[-1] == None:
			AngLo = 90-dsangle
		else:
			AngHi = 90-dsangle
		if i > attempts:
			print('Failed to find path to satisfy')
			finished = True
		elif abs(XZmin-targetPO) < tolPO and gbs[-1]:
			print('Angle of',90-dsangle, 'deg found to give Polar Opening of', XZmin, 'at Y of', passY.y)
			finished = True
		i+=1
	name = 'w'+str(int(dsangle-90))
	makebicolouredwire(gbs, name, colfront=(1.0,0.0,0.0) if fLRdirection == -1 else (0.0,0.0,1.0), colback=(0.7,0.7,0.0), leadcolornodes=dseg+1)
	doc.recompute()
	
	#REPEAT 100 times
	#rpt = repeatwindingpath([[Vector(*gb.pt)  for gb in gbs]], 100)
	#meanthick = evalthick(XZmin, passY.y, [rpt],tw,tth)
	#totrpt = int(100*totthick/meanthick)
	#rpt = repeatwindingpath([[Vector(*gb.pt)  for gb in gbs]], totrpt)
	#ply = Part.show(Part.makePolygon([Vector(pt)  for pt in rpt]), name+'x'+str(totrpt))

Maxsideslipturningfactor = 0.26
combofoldbackmode = 0
mandrelradius = 110  # fc6 file

AngLo = 10
AngHi = 60
maxlength = 6000

alongwire = 0.51
tw = 6.35
tth = 0.18
tolPO = tw/4
targetPO = 35+ 0.5 * tw

qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 500, 570, 350)
qw.setWindowTitle('Generate constant thickness wind')
qsketchplane = freecadutils.qrow(qw, "Sketchplane: ", 15+35*0)
qmeshobject = freecadutils.qrow(qw, "Meshobject: ", 15+35*1 )

qAngLo = freecadutils.qrow(qw, "Min angle: ", 15+35*2, "%.1f" % AngLo)
qAngHi = freecadutils.qrow(qw, "Max angle: ", 15+35*3, "%.1f" % AngHi)

qtargetPO = freecadutils.qrow(qw, "Polar opening: ", 15+35*0, "%.2f" % targetPO, 260)
qtolPO = freecadutils.qrow(qw, "PO tolerance: ", 15+35*1, "%.2f" % tolPO, 260)
qtowwidth = freecadutils.qrow(qw, "Tow width: ", 15+35*2, "%.2f" % tw,260)
qtowthick = freecadutils.qrow(qw, "Tow thickness: ", 15+35*3, "%.2f" % tth,260)
#vlab = QtGui.QLabel("clear above to go one direction", qw)
#vlab.move(20+260, 15+35*3+5)
qtotthick = freecadutils.qrow(qw, "Desired thick: ", 15+35*4, "6.7", 260)
qsideslip = freecadutils.qrow(qw, "Side slip: ", 15+35*5, "0", 260)
qoutputfilament = freecadutils.qrow(qw, "Output name: ", 15+35*4, "w1")
okButton = QtGui.QPushButton("Drive", qw)
okButton.move(180, 15+35*7)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  

qsketchplane.setText(freecadutils.getlabelofselectedsketch())
qmeshobject.setText(freecadutils.getlabelofselectedmesh())

qw.show()