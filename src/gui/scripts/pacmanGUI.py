#!/usr/bin/python
from PyQt5 import QtWidgets, uic, QtCore, QtGui
import sys
import cv2
import threading
import numpy as np


class GUI_API_Wrapper:
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
                self.controller = controller
                self.EStopBtn.clicked.connect(self.toggleEStop)
                self.active = False
                print("SHOWING WINDOW")
                self.show()
                
        def toggleEStop(self):
                self.active = not self.active
                if not self.active:
                    self.EStopBtn.setStyleSheet("background-color: rgb(78, 154, 6);")
                else:
                    self.EStopBtn.setStyleSheet("background: red")
                self.controller.toggleEStop(self.active)

        def displayImageRaw(self, img):
            #h, w, _ = img.shape
            w = self.openGLWidget_5.width()
            h = self.openGLWidget_5.height()
            rz_img = cv2.resize(img, (w, h))
            cv2.imwrite("/home/adam/really.png", rz_img)

            qimg = QtGui.QImage(rz_img, w, h, QtGui.QImage.Format_RGB888)
            self.openGLWidget_5.setPixmap(QtGui.QPixmap(qimg))
            self.openGLWidget_5.show()

# creates the main window; sets the main window of controller
def initGUI(controller):
    GUI_API_Wrapper.app = QtWidgets.QApplication(sys.argv)
    mainWindow = _MainWindow(controller)
    controller.setMainWindow(mainWindow)

# opens and runs the UI
def runGUI():
    print(GUI_API_Wrapper.app)
    return sys.exit(GUI_API_Wrapper.app.exec_())
