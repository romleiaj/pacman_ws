#!/usr/bin/python
from PyQt5 import QtWidgets, uic, QtCore, QtGui
import sys
import threading


class _GUI_API_Wrapper:
        app = None

class MainWindowController:
        def __init__(self):
                self.mainWindow = None

        def setMainWindow(self, mainWindow):
                self.mainWindow = mainWindow
        
        def getMainWindow(self):
                return self.mainWindow

        def getResourceFolder(self):
                return ""


                        
class _MainWindow(QtWidgets.QMainWindow):
        def __init__(self, controller):
                super(_MainWindow, self).__init__()
                resourceFolder = controller.getResourceFolder()
                uic.loadUi("%s/pacman.ui" % (resourceFolder), self)
		self.EStopBtn.clicked.connect(self.toggleEStop)
		self.controller = controller
		self.active = False
                self.show()
                
	def toggleEStop(self):
		self.active = not self.active
		if not self.active:
			self.EStopBtn.setStyleSheet("background-color: rgb(78, 154, 6);")
		else:
			self.EStopBtn.setStyleSheet("background: red")
		self.controller.toggleTStop(self.active)


# creates the main window; sets the main window of controller
def initGUI(controller):
        _GUI_API_Wrapper.app = QtWidgets.QApplication(sys.argv)
        mainWindow = _MainWindow(controller)
        controller.setMainWindow(controller)

# opens and runs the UI
def runGUI():
        return _GUI_API_Wrapper.app.exec_()

        
