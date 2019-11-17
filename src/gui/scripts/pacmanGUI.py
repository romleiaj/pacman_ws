#!/usr/bin/python
from PyQt5 import QtWidgets, uic, QtCore, QtGui
import sys
from rosController import RosController


class Window(QtWidgets.QMainWindow):
	def __init__(self):
		super(Window, self).__init__()
		uic.loadUi('pacman.ui', self)
		self.EStopBtn.clicked.connect(self.toggleEStop)
		self.rosController = RosController(self)
		self.active = False
		self.show()

	def toggleEStop(self):
		self.active = not self.active
		if not self.active:
			self.EStopBtn.setStyleSheet("background-color: rgb(78, 154, 6);")
		else:
			self.EStopBtn.setStyleSheet("background: red")
		# ros(self.active)


if __name__ == '__main__':
	app = QtWidgets.QApplication(sys.argv)
	window = Window()
	exitVar = app.exec_()
	window.rosController.killNode()
	sys.exit(exitVar)



