# -*- coding: utf-8 -*-

# GENERATE A LAYER OF CONSTANT THICKNESS
# First work out what angle is needed to create a given Polar Opening 

from PySide import QtGui, QtCore
from FreeCAD import Vector
import numpy as np
import os, sys
sys.path.append(os.path.join(os.path.split(__file__)[0]))
print(sys.path[-1])

import utils.freecadutils as freecadutils
from utils.directedgeodesic import directedgeodesic, makebicolouredwire
freecadutils.init(App)

def okaypressed():
	sketchplane = freecadutils.findobjectbylabel(qsketchplane.text())
	meshobject = freecadutils.findobjectbylabel(qmeshobject.text())
	outputfilament = qoutputfilament.text()
	targetPO = float(qtargetPO.text())
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
			print('Angle of',90-dsangle, 'deg found to give Polar Opening of', XZmin)
			finished = True
		i+=1
	name = 'w'+str(int(dsangle-90))
	makebicolouredwire(gbs, name, colfront=(1.0,0.0,0.0) if fLRdirection == -1 else (0.0,0.0,1.0), colback=(0.7,0.7,0.0), leadcolornodes=dseg+1)


Maxsideslipturningfactor = 0.26
combofoldbackmode = 0
mandrelradius = 110  # fc6 file

anglefilament = 20
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

qAngLo = freecadutils.qrow(qw, "Min angle: ", 15+35*2, "%.1f" % anglefilament)
qAngHi = freecadutils.qrow(qw, "Max angle: ", 15+35*3, "%.1f" % anglefilament)

qtargetPO = freecadutils.qrow(qw, "Polar opening: ", 15+35*0, "%.2f" % targetPO, 260)
qtolPO = freecadutils.qrow(qw, "PO tolerance: ", 15+35*1, "%.2f" % tolPO, 260)
qtowwidth = freecadutils.qrow(qw, "Tow width: ", 15+35*2, "%.2f" % tw,260)
qtowthick = freecadutils.qrow(qw, "Tow thickness: ", 15+35*3, "%.2f" % tth,260)
#vlab = QtGui.QLabel("clear above to go one direction", qw)
#vlab.move(20+260, 15+35*3+5)
qsideslip = freecadutils.qrow(qw, "Side slip: ", 15+35*4, "0", 260)
qoutputfilament = freecadutils.qrow(qw, "Output name: ", 15+35*4, "w1")
okButton = QtGui.QPushButton("Drive", qw)
okButton.move(180, 15+35*7)
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)  

qsketchplane.setText(freecadutils.getlabelofselectedsketch())
qmeshobject.setText(freecadutils.getlabelofselectedmesh())

qw.show()
