# -*- coding: utf-8 -*-
###################################################################################
#
#  InitGui.py
#  
#  Copyright 2023 Julian Todd <goatchurch> julian@goatchurch.org.uk
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

import filamentwindingwb_locator
filamentwindingWBPath = os.path.dirname(filamentwindingwb_locator.__file__)
filamentwindingWB_icons_path = os.path.join(filamentwindingWBPath,'Resources','icons')

global main_filamentwindingWB_Icon

main_filamentwindingWB_Icon = os.path.join(filamentwindingWB_icons_path , 'CreatePointsObject.svg')

#def myFunc(string):
#    print (string)
#    global act
#    act.setVisible(True)

#mw=Gui.getMainWindow()
#bar=mw.menuBar()
#act=bar.addAction("MyCmd")
#mw.workbenchActivated.connect(myFunc)

####################################################################################
# Initialize the workbench 
class FilamentWindingWorkbench(Workbench):
    global main_filamentwindingWB_Icon

    MenuText = "Filament Winding"
    ToolTip = "FilamentWinding workbench"
    Icon = main_filamentwindingWB_Icon #defined in package.xml
    
    def __init__(self):
        pass

    def Initialize(self):
        "This function is executed when FreeCAD starts"
        import FilamentWindingCmd #needed files for FreeCAD commands
        self.list = ["MeshRemodelCreatePointsObject",
                     "BoxTaskObject"
                    ] # A list of command names created in the line above
        self.appendToolbar("FilamentWinding Commands",self.list[:-3]) # leave settings, validate sketch and merge sketch off toolbar
        self.appendMenu("FilamentWind",self.list) # creates a new menu

    def callback(self,hasUpdate):
        if hasUpdate:
            FreeCAD.Console.PrintMessage("FilamentWinding has an update available via the addon manager.\n")
        #else:
            #FreeCAD.Console.PrintMessage("MeshRemodel up to date\n")
 
    def Activated(self):
        "This function is executed when the workbench is activated"
        #global act
        #act.setVisible(True)
#        import AddonManager as AM
#        if hasattr(AM,"check_updates"):
#            AM.check_updates("MeshRemodel",self.callback)
        return
 
    def Deactivated(self):
        "This function is executed when the workbench is deactivated"

        #FreeCAD will hide our menu and toolbar upon exiting the wb, so we setup a singleshot
        #to unhide them once FreeCAD is finished, 2 seconds later
        from PySide import QtCore
        QtCore.QTimer.singleShot(2000, self.showMenu)
        return 
        
    def showMenu(self):
        from PySide import QtGui
        window = QtGui.QApplication.activeWindow()
        #freecad hides wb toolbars on leaving wb, we unhide ours here to keep it around
        #if the user has it set in parameters to do so
        pg = FreeCAD.ParamGet("User parameter:Plugins/FilamentWinding")
        keep = pg.GetBool('KeepToolbar',True)
        if not keep:
            return
        tb = window.findChildren(QtGui.QToolBar) if window else []
        for bar in tb:
            if "FilamentWinding Commands" in bar.objectName():
                bar.setVisible(True)

    def ContextMenu(self, recipient):
        "This is executed whenever the user right-clicks on screen"
        # "recipient" will be either "view" or "tree"
        #self.appendContextMenu("FilamentWinding",self.list) # add commands to the context menu
 
    def GetClassName(self): 
        # this function is mandatory if this is a full python workbench
        return "Gui::PythonWorkbench"
wb = FilamentWindingWorkbench()
Gui.addWorkbench(wb)





