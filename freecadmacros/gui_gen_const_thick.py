from PySide import QtGui, QtCore
import freecadutils


def okaypressed():
    print("Okay Pressed")


qw = QtGui.QWidget()
qw.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
qw.setGeometry(700, 500, 300, 350)
qw.setWindowTitle('Evaluate thickness')

#qmandrelpaths = freecadutils.qrow(qw, "Winding paths ", 15+35*0, "")
#qy = freecadutils.qrow(qw, "y coord of ring:", 15+35*1)
#qr = freecadutils.qrow(qw, "Radius of ring:", 15+35*2 )
qtowwidth = freecadutils.qrow(qw, "Tow width: ", 15+35*3, "6.35")
qtowthick = freecadutils.qrow(qw, "Tow thickness: ", 15+35*4, "0.18")
okButton = QtGui.QPushButton("Evaluate", qw)
okButton.move(180, 15+35*7)
#qmandrelpaths.setText(freecadutils.getlabelofselectedwire(multiples=True))
#qy.setText(str(passY.Y))
#qr.setText(str(XZmin))
QtCore.QObject.connect(okButton, QtCore.SIGNAL("pressed()"), okaypressed)
qw.show()

