#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import roslib

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

from package import Dialog, Widget

from subprocess import Popen

import rospy, sys, os, time
from geometry_msgs.msg import Twist
from math import pi
import rviz

__path__ = os.path.split(os.path.realpath(__file__))[0]

class MainWindow( QMainWindow ):
	"""docstring for MainWindow"""
	def __init__(self):
		super(MainWindow, self).__init__()

		self.resize(1280, 720)
		if not self.isFullScreen():
			self.showMaximized()

		self.setWindowIcon(QIcon(''.join([__path__, '/icon/icon.jpg'])))
		self.newAction = QAction(QIcon(''.join([__path__, '/icon/new.png'])), "New", self, statusTip = "Close the active window", triggered = self.__newDialog)
		self.openAction = QAction(QIcon(''.join([__path__, '/icon/open.png'])), "Open", self, statusTip = "Open Launch File", triggered = self.__OpenProjectFile)
		self.saveAction = QAction(QIcon(''.join([__path__, '/icon/save.png'])), "Save", self, statusTip = "Close the active window", triggered = self.__saveFile)
		self.cutAction = QAction(QIcon(''.join([__path__, '/icon/cut.png'])), "Cut", self, statusTip = "Close the active window", triggered = self.close)
		self.copyAction = QAction(QIcon(''.join([__path__, '/icon/copy.png'])), "Copy", self, statusTip = "Close the active window", triggered = self.close)
		self.pasteAction = QAction(QIcon(''.join([__path__, '/icon/paste.png'])), "Paste", self, statusTip = "Close the active window", triggered = self.close)
		self.findAction = QAction(QIcon(''.join([__path__, '/icon/find.png'])), "Find", self, statusTip = "Close the active window", triggered = self.__findDialog)
		self.goToCellAction = QAction("Go To", self, statusTip = "Close the active window", triggered = self.close)

		self.rvizPath = None
		self.node = None
		self.saveFlag = True
		self.tmpfilename = None
		self.launchfilename = None
		self.mapfilename = None
		self.MWidget = None
		self.historypath = None

		self.saveAction.setDisabled(True)
		self.cutAction.setDisabled(True)
		self.copyAction.setDisabled(True)
		self.pasteAction.setDisabled(True)
		self.findAction.setDisabled(True)
		self.goToCellAction.setDisabled(True)

		self.l1 = QLabel("Angular Speed: 0.0 rad/s")
		self.l2 = QLabel("Linear Speed: 0.0 m/s")
		
		self.pbar = QProgressBar(self)
		self.timer = QBasicTimer()

		self.__createMenus()
		self.__createToolBar()
		self.createStatusBar()
		self.setWindowTitle('ROS Control Panel')

	def closeEvent(self, e):
		from os import system
		if self.saveFlag != True:
			r = QMessageBox.question(self, self.tr("Quit"), self.tr("The project is not saved. Do you want to close with out save?"), QMessageBox.Cancel | QMessageBox.Save)
			if r == QMessageBox.Save:
				if self.t != None:
					system('killall roslaunch')
					time.sleep(3)
				self.__saveFile()
				return e.accept()
			elif r == QMessageBox.No:
				return e.ignore()
		elif self.t != None:
			system('killall roslaunch')

	# def on_btn1_clicked(self, *args):
	# 	pass

	# def on_btn2_clicked(self, *args):
	# 	pass

	# def on_btn3_clicked(self, *args):
	# 	pass

	def __newDialog(self, *args):
		d = Dialog.New(self, __path__)
		d.show()

	def __createToolBar(self):
		self.fileToolBar = self.addToolBar(self.tr("&File"))
		self.fileToolBar.addAction(self.newAction)
		self.fileToolBar.addAction(self.openAction)
		self.fileToolBar.addAction(self.saveAction)

		self.editToolBar = self.addToolBar(self.tr("&Edit"))
		self.editToolBar.addAction(self.cutAction)
		self.editToolBar.addAction(self.copyAction)
		self.editToolBar.addAction(self.pasteAction)
		self.editToolBar.addSeparator()
		self.editToolBar.addAction(self.findAction)
		self.editToolBar.addAction(self.goToCellAction)

	def __createMenus(self, *args):
		self.t = None
		self.label1 = None
		self.label2 = None
		self.__newProject = QAction(QIcon(''.join([__path__, '/icon/new.png'])), "New", self, statusTip = "Close the active window", triggered = self.__newDialog)
		self.__openLaunch = QAction(QIcon(''.join([__path__, '/icon/open.png'])), "Open", self, statusTip = "Open Launch File", triggered = self.__OpenProjectFile)
		self.__saveProject = QAction(QIcon(''.join([__path__, '/icon/save.png'])), "Save", self, statusTip = "Close the active window", triggered = self.__saveFile)
		self.__setting = QAction(QIcon(''.join([__path__, '/icon/save.png'])), "Setting", self, statusTip = "Close the active window", triggered = self.__saveSetting)
		self.__close = QAction("Cl&ose", self, shortcut = "Ctrl+Q", statusTip = "Close the active window", triggered = self.close)
		self.__copy = QAction(QIcon(''.join([__path__, '/icon/copy.png'])), "Copy", self, statusTip = "Close the active window", triggered = self.close)
		self.__paste = QAction(QIcon(''.join([__path__, '/icon/paste.png'])), "Paste", self, statusTip = "Close the active window", triggered = self.close)
		self.__cut = QAction(QIcon(''.join([__path__, '/icon/cut.png'])), "Cut", self, statusTip = "Close the active window", triggered = self.close)
		self.__selectAll = self.__createAction('Select All')
		self.__about = self.__createAction('About', slot = self.__openAbout)
		self.__menubar = self.menuBar()
		self.__file = self.__menubar.addMenu('&File')
		self.__file.addAction(self.__newProject)
		self.__file.addAction(self.__openLaunch)
		self.__file.addAction(self.__saveProject)
		self.__file.addSeparator()
		self.__file.addAction(self.__setting)
		self.__file.addAction(self.__close)

		self.__edit = self.__menubar.addMenu('&Edit')
		self.__edit.addAction(self.__copy)
		self.__edit.addAction(self.__paste)
		self.__edit.addAction(self.__cut)
		self.__edit.addAction(self.__selectAll)

		self.__setting.setDisabled(True)
		self.__cut.setDisabled(True)
		self.__copy.setDisabled(True)
		self.__paste.setDisabled(True)
		self.__selectAll.setDisabled(True)

		self.__history = self.__menubar.addMenu('&History')
		# self.__history.addAction(self.__copy)

		self.__tools = self.__menubar.addMenu('&Tools')
		# self.__tools.addAction(self.__copy)

		self.__help = self.__menubar.addMenu('&Help')
		self.__help.addAction(self.__about)

	def __createAction(self, text, slot = None, shortcut = None, icon = None, tip = None, checkable = False, signal = "triggered()"):
		newAction = QAction(text, self)
		if icon is not None:
			newAction.setIcon(QIcon(icon))
		if shortcut is not None:
			newAction.setShortcut(shortcut)
		if tip is not None:
			newAction.setToolTip(tip)
			newAction.setStatusTip(tip)
		if slot is not None:
			self.connect(newAction, SIGNAL(signal), slot)
		if checkable:
			newAction.setCheckable(checkable)
		return newAction

	def createStatusBar(self):
		self.statusBar().showMessage("Ready")

	def __saveFile(self, *args):
		import ConfigParser
		config = ConfigParser.RawConfigParser()
		default = '$HOME/Documents'
		config.readfp(open(self.tmpfilename))
		fileName =  QFileDialog.getSaveFileName(self, 'Save File', os.path.expandvars(default) ,"proj files (*.proj *.PROJ)")
		if str(fileName[0]) != '':
			if ('proj' in str(fileName[0])) == False:
				fileName = ''.join([str(fileName[0]), '.proj'])
				with open(fileName, 'wb') as configfile:
					config.write(configfile)
			else:
				fileName = str(fileName[0])
				with open(fileName, 'wb') as configfile:
					config.write(configfile)
			if self.saveFlag != True:
				self.saveFlag = True

	# def __readSetting(self, *args):
	# 	pass

	def __saveSetting(self, *args):
		d = Dialog.Setting(self, __path__)
		d.show()

	def __findDialog(self, *args):
		d = Dialog.Find(self)
		d.show()

	def __openAbout(self, *args):
		QMessageBox.about(self, self.tr("About ROS Control Panel"), self.tr("<h2>ROS Control Panel v 0.1.0-beta</h2>""<p>Copyright &copy; 2016 Software Inc."))

	def timerEvent(self, e):
	
		if self.step >= 100:
		
			self.timer.stop()
			self.statusBar().removeWidget(self.pbar)
			self.MWidget = Widget.MainWidget(self, __path__, self.rvizPath, self.node)
			self.saveAction.setDisabled(False)
			# self.__setting.setDisabled(False)
			# self.cutAction.setDisabled(False)
			# self.copyAction.setDisabled(False)
			# self.pasteAction.setDisabled(False)
			self.findAction.setDisabled(False)
			# self.goToCellAction.setDisabled(False)
			self.setCentralWidget( self.MWidget )
			return
		
		if self.step == 5:
			if self.mapfilename != '':
				self.t = Popen(['roslaunch', self.launchfilename, 'map:=' + os.path.split(self.mapfilename)[1]])
			else:
				self.t = Popen(['roslaunch', self.launchfilename])

		self.step = self.step + 2.5
		self.pbar.setValue(self.step)

	def loadLaunchFilePrograss(self):
		self.step = 0
		self.statusBar().addPermanentWidget(self.pbar)
		if self.timer.isActive():
			self.timer.stop()
		else:
			self.timer.start(50, self)

	def launch(self, launchfilename, mapfilename):
		self.launchfilename = launchfilename
		self.mapfilename = mapfilename
		from os import system
		if self.mapfilename != '':
			if self.t != None:
				system('killall roslaunch')
				self.statusBar().removeWidget(self.l1)
				self.statusBar().removeWidget(self.l2)
				time.sleep(3)
			self.loadLaunchFilePrograss()
		else:
			if self.t != None:
				system('killall roslaunch')
				self.statusBar().removeWidget(self.l1)
				self.statusBar().removeWidget(self.l2)
				time.sleep(3)
			self.loadLaunchFilePrograss()

	def __OpenProjectFile(self, *args):
		import ConfigParser
		from os import system
		config = ConfigParser.RawConfigParser()
		default = '$HOME/Documents'
		fileName = QFileDialog.getOpenFileName(self, "Open Project File", os.path.expandvars(default), "Project Files (*.proj *.PROJ)")
		if str(fileName[0]) != '':
			config.readfp(open(str(fileName[0])))
			self.rvizPath = config.get('filepath', 'rvizpath')
			self.node = config.get('setting', 'node')
			self.launchfilename = config.get('filepath', 'launchpath')
			enableMap = config.get('setting', 'enablemap')
			self.mapfilename = config.get('filepath', 'mapfilepath')
			if self.saveFlag != True:
				self.saveFlag = True
			if int(enableMap) != 0:
				if self.t != None:
					system('killall roslaunch')
					self.statusBar().removeWidget(self.l1)
					self.statusBar().removeWidget(self.l2)
					time.sleep(3)
				self.loadLaunchFilePrograss()
			else:
				if self.t != None:
					system('killall roslaunch')
					self.statusBar().removeWidget(self.l1)
					self.statusBar().removeWidget(self.l2)
					time.sleep(3)
				self.loadLaunchFilePrograss()
				
def main(*args):

	app = QApplication(sys.argv)
	MWindow = MainWindow()
	sys.exit(app.exec_())

if __name__ == '__main__':
	main()