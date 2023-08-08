# -*- coding: utf-8 -*-
###################################################################################
#
#  MeshRemodelCmd.py
#  
#  Copyright 2019 Mark Ganson <TheMarkster> mwganson at gmail
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#  
###################################################################################

__title__   = "MeshRemodel"
__author__  = "Mark Ganson <TheMarkster>"
__url__     = "https://github.com/mwganson/MeshRemodel"
__date__    = "2023.08.10"
__version__ = "1.0"
version = 1.0

import FreeCAD, FreeCADGui, Part, os, math
from PySide import QtCore, QtGui
import Draft, DraftGeomUtils, DraftVecUtils
import time


if FreeCAD.GuiUp:
    from FreeCAD import Gui

__dir__ = os.path.dirname(__file__)
iconPath = os.path.join( __dir__, 'Resources', 'icons' )
keepToolbar = False
windowFlags = QtCore.Qt.WindowTitleHint | QtCore.Qt.WindowCloseButtonHint #no ? in title bar
global_picked = [] #picked points list for use with selection by preselection observer
FC_VERSION = float(FreeCAD.Version()[0]) + float(FreeCAD.Version()[1]) #e.g. 0.20, 0.18, 1.??

def fixTip(tip):
    if FC_VERSION >= 0.20:
        return tip.replace("\n","<br/>")
    else:
        return tip




######################################################################################
# geometry utilities

class MeshRemodelGeomUtils(object):
    """Geometry Utilities"""

    #progress bar on status bar with cancel button
    class MRProgress:
        def __init__(self):
            self.pb = None
            self.btn = None
            self.bar = None
            self.bCanceled = False
            self.value = 0
            self.total = 0
            self.mw = FreeCADGui.getMainWindow()
            self.lastUpdate = time.time()

        def makeProgressBar(self,total=0,buttonText = "Cancel",tooltip = "Cancel current operation",updateInterval = .5):
            """total is max value for progress bar, mod = number of updates you want"""
            self.btn = QtGui.QPushButton(buttonText)
            self.btn.setToolTip(tooltip)
            self.btn.clicked.connect(self.on_clicked)
            self.pb = QtGui.QProgressBar()
            self.bar = self.mw.statusBar()
            self.bar.addWidget(self.pb)
            self.bar.addWidget(self.btn)
            self.btn.show()
            self.pb.show()
            self.pb.reset()
            self.value = 0
            self.pb.setMinimum(0)
            self.updateInterval = updateInterval
            self.pb.setMaximum(total);
            self.total = total
            self.pb.setFormat("%v/%m")
            self.bAlive = True #hasn't been killed yet
            self.bCanceled = False

        def on_clicked(self):
            self.bCanceled = True
            self.killProgressBar()

        def isCanceled(self):
            self.value += 1
            timeNow = time.time()
            if timeNow - self.lastUpdate >= self.updateInterval:
                self.lastUpdate = timeNow
                self.pb.setValue(self.value)
                FreeCADGui.updateGui()
            if self.mw.isHidden():
                self.bCanceled = True
                self.killProgressBar()
            return self.bCanceled

        def killProgressBar(self):
            if self.bAlive: #check if it has already been removed before removing
                self.bar.removeWidget(self.pb)
                self.bar.removeWidget(self.btn)
                self.bAlive = False
            self.pb.hide()
            self.btn.hide()
            self.value = 0
            self.total = 0

#source for this block of code: https://stackoverflow.com/questions/9866452/calculate-volume-of-any-tetrahedron-given-4-points
#4 points are coplanar if the tetrahedron defined by them has volume = 0
##################################################################
    def determinant_3x3(self,m):
        """helper for isCoplanar()"""
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                m[1][0] * (m[0][1] * m[2][2] - m[0][2] * m[2][1]) +
                m[2][0] * (m[0][1] * m[1][2] - m[0][2] * m[1][1]))


    def subtract(self, a, b):
        """ helper for isCoplanar()"""
        return (a[0] - b[0],
                a[1] - b[1],
                a[2] - b[2])

    def tetrahedron_calc_volume(self, a, b, c, d):
        """helper for isCoplanar()"""
        return (abs(self.determinant_3x3((self.subtract(a, b),
                                 self.subtract(b, c),
                                 self.subtract(c, d),
                                 ))) / 6.0)

#a = [0.0, 0.0, 0.0]
#d = [2.0, 0.0, 0.0]
#c = [0.0, 2.0, 0.0]
#b = [0.0, 0.0, 2.0]

#print(tetrahedron_calc_volume(a, b, c, d))
    def isCoplanar(self, trio, pt, tol=1e-3):
        """ isCoplanar(trio, pt, tol=1e-3)
            trio is a 3-element list of vectors, pt is a vector to test, tol is tolerance, return True if all 4 are coplanar
            test is done by creating a tetrahedron and testing its volume against tol
            a tetrahedron from 4 coplanar points should have volume ~= 0
        """
        if len(trio) != 3:
            raise Exception("MeshRemodel GeomUtils Error: isCoplanar() trio parameter must be list of 3 vectors")
        A,B,C,D = trio[0],trio[1],trio[2],pt
        vol = self.tetrahedron_calc_volume(A,B,C,D)

        if vol <= tol:
            return True
        return False

    def hasPoint(self,pt,lis,tol):
        """hasPoint(pt,lis,tol)"""
        for l in lis:
            if self.isSamePoint(pt,l,tol):
                return True
        return False

    def isSamePoint(self,A,B,tol):
        """isSamePoint(A,B,tol)"""
        dis = self.dist(A,B)
        if dis < tol:
            return True
        return False

    def midpoint(self, A, B):
        """ midpoint(A, B)
            A,B are vectors, return midpoint"""
        mid = FreeCAD.Base.Vector()
        mid.x = (A.x + B.x)/2.0
        mid.y = (A.y + B.y)/2.0
        mid.z = (A.z + B.z)/2.0
        return mid

    def dist(self, p1, p2):
        """ dist (p1, p2)
            3d distance between vectors p1 and p2"""
        return self.getDistance3d(p1[0],p1[1],p1[2],p2[0],p2[1],p2[2])

    def getDistance3d(self, x1, y1, z1, x2, y2, z2):
        """ getDistance3d(x1, y1, z1, x2, y2, z2)
            3d distance between x1,y1,z1 and x2,y2,z2 float parameters"""
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)

    def sortPoints(self,pts):
        """ sortPoints(pts)
            sort pts, a list of vectors, according to distance from one point to the next
            pts[0] is taken first, then the nearest point to it is placed at pts[1]
            then pts[1] is used to find the nearest point to it and placed at pts[2], and so on
        """
        newList = [pts[0]]
        for ii in range(0, len(pts)):
            newList.extend([self.nearestPoint(newList[ii],pts,newList)])
        return newList

    def flattenPoints(self, pts, align_plane):
        """ project points to align_plane.
            pts is list of Part.Vertex objects
            align_plane is part::plane object or 
            can be any object with a face (face1 will be used)
            returns: new list of Part.Vertex objects on the plane"""
        plane = align_plane.Shape.Faces[0]
        normal = plane.normalAt(0,0)
        base = align_plane.Shape.Vertexes[0].Point
        verts = []
        flatPts = [] #eliminate duplicate points
        pb = gu.MRProgress()
        pb.makeProgressBar(len(pts),buttonText = "Cancel Phase1/2",tooltip="Cancel projecting points to plane")
        for p in pts:
            fpt = p.Point.projectToPlane(base,normal)
            if not self.hasPoint(fpt,flatPts,1e-7):
                flatPts.append(fpt)
            if pb.isCanceled():
                FreeCAD.Console.PrintWarning("Phase 1 / 2 canceled, object may be incomplete.\n")
                break
        pb.killProgressBar()

        pb.makeProgressBar(len(flatPts),"Cancel Phase2/2")
        for p in flatPts:
            verts.append(Part.Vertex(p))
            if pb.isCanceled():
                FreeCAD.Console.PrintWarning("Phase 2 / 2 canceled, object may be incomplete.\n")
                break
        pb.killProgressBar()
        return verts

    def nearestPoint(self, pt, pts, exclude):
        """ nearestPoint(pt, pts, exclude)
            pt is a vector, pts a list of vectors
            exclude is a list of vectors to exclude from process
            return nearest point to pt in pts and not in exclude"""
        if len(pts) == 0: #should never happen
            raise Exception("MeshRemodel GeomUtils Error: nearestPoint() pts length = 0\n")
        nearest = pts[0]
        d = 10**100
        for p in pts:
            if p in exclude:
                continue
            dis = self.dist(pt, p)
            if dis < d:
                d = dis
                nearest = p
        return nearest

    def isColinear(self,A,B,C):
        """ isColinear(A, B, C)
            determine whether vectors A,B,C are colinear """
        return DraftVecUtils.isColinear([A,B,C])

    def incenter(A,B,C):
        """ incenter(A, B, C)
            return incenter (vector) of triangle at vectors A,B,C 
            incenter is center of circle fitting inside the triangle
            tangent to all 3 sides
            raises exception if A,B,C are colinear
        """

        if self.isColinear(A,B,C):
            raise Exception("MeshRemodel Error: incenter() A,B,C are colinear")

        Ax,Ay,Az = A[0],A[1],A[2] 
        Bx,By,Bz = B[0],B[1],B[2]
        Cx,Cy,Cz = C[0],C[1],C[2]

        a = self.dist(B,C)
        b = self.dist(C,A)
        c = self.dist(A,B)
        s = a+b+c

        Ix = (a*Ax+b*Bx+c*Cx)/s
        Iy = (a*Ay+b*By+c*Cy)/s
        Iz = (a*Az+b*Bz+c*Cz)/s
        I = FreeCAD.Base.Vector(Ix,Iy,Iz)
        return I

    def inradius(A,B,C):
        """ inradius(A, B, C)
            return inradius of triangle A,B,C 
            this is radius of incircle, the circle that
            fits inside the triangle, tangent to all 3 sides
        """
        return self.dist(A, self.incenter(A,B,C))

#python code below was adapted from this javascript code
#from here: https://gamedev.stackexchange.com/questions/60630/how-do-i-find-the-circumcenter-of-a-triangle-in-3d
#in a question answered by user greenthings

#function circumcenter(A,B,C) {
#    var z = crossprod(subv(C,B),subv(A,B));
#    var a=vlen(subv(A,B)),b=vlen(subv(B,C)),c=vlen(subv(C,A));
#    var r = ((b*b + c*c - a*a)/(2*b*c)) * outeradius(a,b,c);
#    return addv(midpoint(A,B),multv(normaliz(crossprod(subv(A,B),z)),r));
#}

#function outeradius(a,b,c) { /// 3 lens
#    return (a*b*c) / (4*sss_area(a,b,c));
#}

#function sss_area(a,b,c) {
#    var sp = (a+b+c)*0.5;
#    return Math.sqrt(sp*(sp-a)*(sp-b)*(sp-c));
#    //semi perimeter
#}

    def circumcenter(self,A,B,C):
        """ circumcenter(A, B, C)
            return the circumcenter of triangle A,B,C
            the circumcenter is the circle that passes through
            all 3 of the triangle's vertices
            raises exception if A,B,C are colinear
        """
        if self.isColinear(A,B,C):
            raise Exception("MeshRemodel Error: circumcenter() A,B,C are colinear")

        z = C.sub(B).cross(A.sub(B))
        a = A.sub(B).Length
        b = B.sub(C).Length
        c = C.sub(A).Length
        r = ((b*b + c*c - a*a)/(2*b*c)) * self.outerradius(a,b,c)
        return  A.sub(B).cross(z).normalize().multiply(r).add(self.midpoint(A,B))

    def outerradius(self, a, b, c):
        """ helper for circumcenter()"""
        return (a*b*c) / (4*self.sss_area(a,b,c))

    def sss_area(self,a,b,c): #semiperimeter
        """ helper for circumcenter()"""
        sp = (a+b+c)*0.5;
        return math.sqrt(sp*(sp-a)*(sp-b)*(sp-c))

    def circumradius(self, A,B,C):
        """ circumradius(A, B, C)
            returns the radius of circumcircle of triangle A,B,C
            A,B,C are vectors
            the circumcircle is the circle passing through A, B, and C
        """
        return self.dist(A, self.circumcenter(A,B,C))



gu = MeshRemodelGeomUtils()
#######################################################################################
# Settings

class MeshRemodelSettingsCommandClass(object):
    """Settings"""

    def __init__(self):
        pass

    def GetResources(self):
        return {'Pixmap'  : os.path.join( iconPath , 'Settings.svg') , # the name of an icon file available in the resources
            'MenuText': "&Settings" ,
            'ToolTip' : "Workbench settings dialog"}
 
    def Activated(self):
        doc = FreeCAD.ActiveDocument
        from PySide import QtGui
        window = QtGui.QApplication.activeWindow()
        pg = FreeCAD.ParamGet("User parameter:Plugins/MeshRemodel")
        keep = pg.GetBool('KeepToolbar',False)
        point_size = pg.GetFloat("PointSize", 4.0)
        line_width = pg.GetFloat("LineWidth", 5.0)
        prec = pg.GetInt("SketchRadiusPrecision", 1)
        coplanar_tol = pg.GetFloat("CoplanarTolerance",.01)
        wireframe_tol = pg.GetFloat("WireFrameTolerance",.01)
        items=[("","*")[keep]+"Keep the toolbar active",
            ("","*")[not keep]+"Do not keep the toolbar active",
            "Change point size ("+str(point_size)+")",
            "Change line width ("+str(line_width)+")",
            "Change sketch radius precision ("+str(prec)+")",
            "Change coplanar tolerance ("+str(coplanar_tol)+")",
            "Change wireframe tolerance("+str(wireframe_tol)+")",
            "Cancel"]
        item,ok = QtGui.QInputDialog.getItem(window,'Mesh Remodel v'+__version__,'Settings\n\nSelect the settings option\n',items,0,False,windowFlags)
        if ok and item == items[-1]:
            return
        elif ok and item == items[0]:
            keep = True
            pg.SetBool('KeepToolbar', keep)
        elif ok and item==items[1]:
            keep = False
            pg.SetBool('KeepToolbar', keep)
        elif ok and item==items[2]:
            new_point_size,ok = QtGui.QInputDialog.getDouble(window,"Point size", "Enter point size", point_size,1,50,2)
            if ok:
                pg.SetFloat("PointSize", new_point_size)
        elif ok and item==items[3]:
            new_line_width,ok = QtGui.QInputDialog.getDouble(window,"Line width", "Enter line width", line_width,1,50,2)
            if ok:
                pg.SetFloat("LineWidth", new_line_width)
        elif ok and item==items[4]:
            new_prec, ok = QtGui.QInputDialog.getInt(window,"Sketch Radius Precision", "\n\
-1 = no radius constraints\n\
0 = radius constraints\n\
1-12 = decimal points to round to when constraining\n\n\
Enter new sketch radius precision", prec, -1,12,1,flags=windowFlags)
            if ok:
                pg.SetInt("SketchRadiusPrecision", new_prec)
        elif ok and item==items[5]:
            new_coplanar_tol, ok = QtGui.QInputDialog.getDouble(window,"Coplanar tolerance", "Enter coplanar tolerance\n(Used when creating coplanar points.  Increase if some points are missing.)", coplanar_tol,.0000001,1,8)
            if ok:
                pg.SetFloat("CoplanarTolerance", new_coplanar_tol)
        elif ok and item==items[6]:
            new_wireframe_tol, ok = QtGui.QInputDialog.getDouble(window,"Wireframe tolerance", "Enter wireframe tolerance\n(Used when creating wireframes to check if 2 points are the same.)", wireframe_tol,.0000001,1,8)
            if ok:
                pg.SetFloat("WireFrameTolerance", new_wireframe_tol)
        return

    def IsActive(self):
        return True

#end settings class


####################################################################################
# Create the Mesh Remodel Points Object

class MeshRemodelCreatePointsObjectCommandClass(object):
    """Create Points Object command"""

    def __init__(self):
        self.mesh = None

    def GetResources(self):
        return {'Pixmap'  : os.path.join( iconPath , 'CreatePointsObject.svg') ,
            'MenuText': "Create points &object" ,
            'ToolTip' : "Create the points object from selected Mesh or Points cloud object\n\
(Ctrl + Click to make mesh partially transparent and non-selectable.\n\
Non-selectability can be reversed in the mesh object's view tab in the property view.)\n\
"}
 
    def Activated(self):
        modifiers = QtGui.QApplication.keyboardModifiers()
        doc = FreeCAD.ActiveDocument
        pg = FreeCAD.ParamGet("User parameter:Plugins/MeshRemodel")
        point_size = pg.GetFloat("PointSize",4.0)
        QtGui.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        doc.openTransaction("Create points object")

        pts=[]
        if hasattr(self.mesh,"Mesh"):
            meshpts = self.mesh.Mesh.Points
            for m in meshpts:
                p = Part.Point(m.Vector)
                pts.append(p.toShape())
        elif hasattr(self.mesh,"Points") and hasattr(self.mesh.Points,"Points"): #points cloud
            meshpts = self.mesh.Points.Points
            for m in meshpts:
                p = Part.Point(m)
                pts.append(p.toShape())

        Part.show(Part.makeCompound(pts),"MR_Points")
        doc.ActiveObject.ViewObject.PointSize = point_size
        doc.recompute()
        if modifiers == QtCore.Qt.ControlModifier:
            self.mesh.ViewObject.Transparency = 75
            self.mesh.ViewObject.Selectable = False
        doc.commitTransaction()
        doc.recompute()
        QtGui.QApplication.restoreOverrideCursor()
        return

    def IsActive(self):
        if not FreeCAD.ActiveDocument:
            return False
        sel = Gui.Selection.getSelectionEx()
        if len(sel) == 0:
            return False
        elif "Mesh.Feature" not in str(type(sel[0].Object)) and not hasattr(sel[0].Object,"Points") and not hasattr(sel[0].Object.Points,"Points"):
            return False
        else:
            self.mesh = sel[0].Object
        return True

# end create points class

##################################################################################################
# initialize

def initialize():
    if FreeCAD.GuiUp:
        Gui.addCommand("MeshRemodelCreatePointsObject", MeshRemodelCreatePointsObjectCommandClass())

initialize()
