#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
Dialog
"""

import sys, Config, os
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

class Setting(QDialog):
	"""设置对话框，用于修改和保存设置内容"""
	def __init__(self, parent, path):
		super(Setting, self).__init__(parent)
		self.path = path
		self.parent = parent
		self.configPath = ''.join([self.path, '/config.cfg'])
		self.Config = Config.Config(self.configPath)

		self.cancelButton = QPushButton('&Cancel')
		self.saveButton = QPushButton('&Save')

		self.labelLaunchFilePath = QLabel('Launch file:')
		self.launchFilePath = QLineEdit()
		self.launchFilePath.textChanged[str].connect(self.__onChanged)
		self.browseButton = QPushButton('&Browse')

		self.labelNode = QLabel('Node:')
		self.node = QLineEdit()
		self.node.textChanged[str].connect(self.__onChanged)

		self.labelRvizFilePath = QLabel('Rviz file:')
		self.rvizFilePath = QLineEdit()
		self.rvizFilePath.textChanged[str].connect(self.__onChanged)
		self.rvizBrowseButton = QPushButton('&Browse')

		self.enableRvizCheckBox = QCheckBox('Enable Rviz', self)
		self.enableRviz = 0
		self.enableRvizCheckBox.stateChanged.connect(self.__enableRviz)
		self.enableControllerCheckBox = QCheckBox('Enable Controller', self)
		self.enableController = 0
		self.enableControllerCheckBox.stateChanged.connect(self.__enableController)
		self.enableMapCheckBox = QCheckBox('Enable Map', self)
		self.enableMap = 0
		self.enableMapCheckBox.stateChanged.connect(self.__enableMap)
		# self.enableMapCheckBox.toggle()
		self.cb_3 = QCheckBox('Show title', self)
		# cb_3.toggle()
		self.cancelButton.setDefault(True)
		self.saveButton.setDisabled(True)

		self.connect(self.browseButton, SIGNAL("clicked()"), self.__browseLaunchFile)
		self.connect(self.rvizBrowseButton, SIGNAL("clicked()"), self.__browseRvizFile)
		self.connect(self.saveButton, SIGNAL("clicked()"), self.__saveSetting)
		self.connect(self.cancelButton, SIGNAL("clicked()"), self.reject)
		
		self.__init()
		self.setFixedSize(450, 250)
		self.setWindowTitle('Setting')
		self.setWindowModality(Qt.ApplicationModal)

	def __init(self):
		self.topLayout = QGridLayout()
		self.topLayout.addWidget(self.labelLaunchFilePath, 0, 0)
		self.topLayout.addWidget(self.labelRvizFilePath, 1, 0)
		self.topLayout.addWidget(self.labelNode, 2, 0)
		self.topLayout.addWidget(self.launchFilePath, 0, 1)
		self.topLayout.addWidget(self.rvizFilePath, 1, 1)
		self.topLayout.addWidget(self.node, 2, 1)
		self.topLayout.addWidget(self.browseButton, 0, 2)
		self.topLayout.addWidget(self.rvizBrowseButton, 1, 2)

		self.bottomLeftLayout = QVBoxLayout()
		self.bottomLeftLayout.addWidget(self.enableRvizCheckBox)
		self.bottomLeftLayout.addWidget(self.enableControllerCheckBox)

		self.bottomRightLayout = QVBoxLayout()
		self.bottomRightLayout.addWidget(self.enableMapCheckBox)
		self.bottomRightLayout.addWidget(self.cb_3)

		self.bottomLayout = QHBoxLayout()
		self.bottomLayout.addLayout(self.bottomLeftLayout)
		self.bottomLayout.addLayout(self.bottomRightLayout)

		self.buttonLayout = QHBoxLayout()
		self.buttonLayout.addStretch()
		self.buttonLayout.addWidget(self.cancelButton)
		self.buttonLayout.addWidget(self.saveButton)

		self.mainLayout = QVBoxLayout()
		self.mainLayout.addLayout(self.topLayout)
		self.mainLayout.addLayout(self.bottomLayout)
		self.mainLayout.addLayout(self.buttonLayout)

		self.setLayout(self.mainLayout)
	
	def __saveSetting(self):
		if os.path.isfile(self.rvizFilePath.text()) != True or os.path.isfile(self.launchFilePath.text()) != True:
			r = QMessageBox.warning(self, self.tr("Warning"), self.tr("File not exists!"), QMessageBox.Yes)
			if r == QMessageBox.Yes:
				return
		else:
			self.Config.saveSetting(rvizpath = self.rvizFilePath.text(), launchpath = self.launchFilePath.text(), node = self.node.text(), name = 'ROS Control Panel', enableRviz = self.enableRviz, enableController = self.enableController, configFilePath = self.configPath)
			self.accept()

	def __browseLaunchFile(self):
		if self.parent.historypath == None:
			openpath = os.path.expandvars('$HOME')
		else:
			if os.path.isfile(self.parent.historypath):
				openpath = os.path.dirname(self.parent.historypath)
			elif os.path.isdir(self.parent.historypath):
				openpath = self.parent.historypath
		fileName = QFileDialog.getOpenFileName(self, "Open Launch File", openpath, "Launch Files (*.launch *.LAUNCH)")
		if str(fileName[0]) != '':
			self.parent.historypath = str(fileName[0])
			self.launchFilePath.setText(str(fileName[0]))
			if self.launchFilePath.text() != '' and self.rvizFilePath.text() != '' and self.node.text() != '':
				self.saveButton.setDisabled(False)

	def __enableMap(self):
		self.__onChanged(self)
		if self.enableMapCheckBox.isChecked():
			r = QMessageBox.question(self, self.tr("Conform"), self.tr("Enable map need restart roslaunch. Do you want to restart roslaunch"), QMessageBox.Yes | QMessageBox.No)
			if r == QMessageBox.Yes:
				self.enableMap = 1
			elif r == QMessageBox.No:
				self.enableMap = 0
		else:
			self.enableMap = 0

	def __enableRviz(self):
		self.__onChanged(self)
		if self.enableRvizCheckBox.isChecked():
			self.enableRviz = 1
		else:
			self.enableRviz = 0

	def __enableController(self):
		self.__onChanged(self)
		if self.enableControllerCheckBox.isChecked():
			self.enableController = 1
		else:
			self.enableController = 0

	def __onChanged(self, text):
		if self.launchFilePath.text() != '' and self.rvizFilePath.text() != '' and self.node.text() != '':
			self.saveButton.setDisabled(False)
		else:
			self.saveButton.setDisabled(True)

	def __browseRvizFile(self):
		if self.parent.historypath == None:
			openpath = os.path.expandvars('$HOME')
		else:
			if os.path.isfile(self.parent.historypath):
				openpath = os.path.dirname(self.parent.historypath)
			elif os.path.isdir(self.parent.historypath):
				openpath = self.parent.historypath
		fileName = QFileDialog.getOpenFileName(self, "Open Launch File", openpath, "Rviz Files (*.rviz *.RVIZ)")
		if str(fileName[0]) != '':
			self.parent.historypath = str(fileName[0])
			self.rvizFilePath.setText(str(fileName[0]))
			if self.launchFilePath.text() != '' and self.rvizFilePath.text() != '' and self.node.text() != '':
				self.saveButton.setDisabled(False)
			else:
				self.saveButton.setDisabled(True)

class Find(QDialog):
	"""docstring for Find"""
	def __init__(self, parent):
		super(Find, self).__init__(parent)
		self.parent = parent
		self.label = QLabel('Find:')
		self.lineEdit = QLineEdit()
		self.lineEdit.textChanged[str].connect(self.__onChanged)
		self.label.setBuddy(self.lineEdit)
		self.caseCheckBox = QCheckBox('Match &case')
		self.backwardCheckBox = QCheckBox('Search &backward')
		self.findButton = QPushButton('&Find')
		self.findButton.setDefault(True)
		self.findButton.setEnabled(False)
		self.closeButton = QPushButton('Close')
		self.connect(self.closeButton, SIGNAL("clicked()"), self.close)
		self.init()
		self.setWindowTitle('Find')
		self.setWindowModality(Qt.ApplicationModal)
		self.setFixedSize(350, 100)

	def init(self):
		self.topLeftLayout = QHBoxLayout()
		self.topLeftLayout.addWidget(self.label)
		self.topLeftLayout.addWidget(self.lineEdit)

		self.leftLayout = QVBoxLayout()
		self.leftLayout.addLayout(self.topLeftLayout)
		self.leftLayout.addWidget(self.caseCheckBox)
		self.leftLayout.addWidget(self.backwardCheckBox)

		self.rightLayout = QVBoxLayout()
		self.rightLayout.addWidget(self.findButton)
		self.rightLayout.addWidget(self.closeButton)
		self.rightLayout.addStretch()

		self.mainLayout = QHBoxLayout()
		self.mainLayout.addLayout(self.leftLayout)
		self.mainLayout.addLayout(self.rightLayout)
		self.setLayout(self.mainLayout)

	def __onChanged(self, text):
		if self.lineEdit.text() != '':
			self.findButton.setDisabled(False)
		else:
			self.findButton.setDisabled(True)

class New(QDialog):
	"""docstring for New"""
	def __init__(self, parent, path):
		super(New, self).__init__(parent)
		self.parent = parent
		self.path = path
		self.configPath = ''.join([self.path, '/config.cfg'])
		self.Config = Config.Config(self.configPath)

		self.cancelButton = QPushButton('&Cancel')
		self.saveButton = QPushButton('C&ontinue')
		self.cancelButton.setDefault(True)
		self.saveButton.setDisabled(True)

		self.baseTabWidget = QTabWidget()
		self.baseTabWidget.setTabPosition(QTabWidget.North)
		
		self.tab1 = BaseTabWidget(self, self.Config)
		self.tab2 = AdvancedTabWidget(self, self.Config)

		self.baseTabWidget.addTab(self.tab1, self.tr("&Base Information"))
		self.baseTabWidget.addTab(self.tab2, self.tr("    &Advanced    "))

		self.connect(self.saveButton, SIGNAL("clicked()"), self.__saveandlaunch)
		self.connect(self.cancelButton, SIGNAL("clicked()"), self.reject)
		
		self.__init()
		self.setFixedSize(480, 360)
		self.setWindowTitle('New Project')
		self.setWindowModality(Qt.ApplicationModal)

	def __init(self):
		self.topLayout = QVBoxLayout()
		self.topLayout.addWidget(self.baseTabWidget)

		self.bottomLayout = QHBoxLayout()
		self.bottomLayout.addStretch()
		self.bottomLayout.addWidget(self.cancelButton)
		self.bottomLayout.addWidget(self.saveButton)

		self.mainLayout = QVBoxLayout()
		self.mainLayout.addLayout(self.topLayout)
		self.mainLayout.addLayout(self.bottomLayout)

		self.setLayout(self.mainLayout)

	def __saveandlaunch(self):
		import random
		r = random.Random()
		if self.tab2.enableMap == 0:
			if os.path.isfile(self.tab1.launchFilePath.text()) != True or os.path.isfile(self.tab1.rvizFilePath.text()) != True:
				if os.path.isfile(self.tab1.launchFilePath.text()) != True:
					r = QMessageBox.warning(self, self.tr("Warning"), self.tr("Launch file is not exist!"), QMessageBox.Yes)
					if r == QMessageBox.Yes:
						return
				elif os.path.isfile(self.tab1.rvizFilePath.text()) != True:
					r = QMessageBox.warning(self, self.tr("Warning"), self.tr("Rviz file is not exist!"), QMessageBox.Yes)
					if r == QMessageBox.Yes:
						return
			else:
				self.parent.tmpfilename = '/tmp/tmp.' + str(int(r.random() * 100000000000))
				self.parent.saveFlag = False
				self.Config.saveSetting(rvizpath = self.tab1.rvizFilePath.text(), launchpath = self.tab1.launchFilePath.text(), node = self.tab1.node.text(), name = 'ROS Control Panel', mapFilePath = '', enableMap = 0, enableRviz = 0, enableController = 0, configFilePath = self.parent.tmpfilename)
				self.accept()
				self.parent.rvizPath = self.tab1.rvizFilePath.text()
				self.parent.node = self.tab1.node.text()
				self.parent.launch(launchfilename = self.tab1.launchFilePath.text(), mapfilename = '')
		else:
			if os.path.isfile(self.tab1.launchFilePath.text()) != True or os.path.isfile(self.tab1.rvizFilePath.text()) != True or os.path.isfile(self.tab2.mapFilePath.text()) != True:
				if os.path.isfile(self.tab1.launchFilePath.text()) != True:
					r = QMessageBox.warning(self, self.tr("Warning"), self.tr("Launch file is not exist!"), QMessageBox.Yes)
					if r == QMessageBox.Yes:
						return
				elif os.path.isfile(self.tab1.rvizFilePath.text()) != True:
					r = QMessageBox.warning(self, self.tr("Warning"), self.tr("Rviz file is not exist!"), QMessageBox.Yes)
					if r == QMessageBox.Yes:
						return
				elif os.path.isfile(self.tab2.mapFilePath.text()) != True:
					r = QMessageBox.warning(self, self.tr("Warning"), self.tr("Map file is not exist!"), QMessageBox.Yes)
					if r == QMessageBox.Yes:
						return
			else:
				# /tmp/tmp
				self.parent.tmpfilename = '/tmp/tmp.' + str(int(r.random() * 100000000000))
				self.parent.saveFlag = False
				self.Config.saveSetting(rvizpath = self.tab1.rvizFilePath.text(), launchpath = self.tab1.launchFilePath.text(), node = self.tab1.node.text(), name = 'ROS Control Panel', mapFilePath = self.tab2.mapFilePath.text(), enableMap = 1, enableRviz = 0, enableController = 0, configFilePath = self.parent.tmpfilename)
				self.accept()
				self.parent.rvizPath = self.tab1.rvizFilePath.text()
				self.parent.node = self.tab1.node.text()
				self.parent.launch(launchfilename = self.tab1.launchFilePath.text(), mapfilename = self.tab2.mapFilePath.text())

class BaseTabWidget(QDialog):
	"""docstring for BaseTabWidget"""
	def __init__(self, parent, config):
		super(BaseTabWidget, self).__init__(parent)
		self.config = config
		self.parent = parent

		self.labelLaunchFilePath = QLabel('Launch file:')
		self.launchFilePath = QLineEdit()
		self.launchFilePath.textChanged[str].connect(self.__onChanged)
		self.browseButton = QPushButton('&Browse')

		self.labelNode = QLabel('Node:')
		self.node = QLineEdit()
		self.node.setText('/cmd_vel')
		self.node.textChanged[str].connect(self.__onChanged)

		self.labelRvizFilePath = QLabel('Rviz file:')
		self.rvizFilePath = QLineEdit()
		self.rvizFilePath.textChanged[str].connect(self.__onChanged)
		self.rvizBrowseButton = QPushButton('&Browse')

		self.connect(self.browseButton, SIGNAL("clicked()"), self.__browseLaunchFile)
		self.connect(self.rvizBrowseButton, SIGNAL("clicked()"), self.__browseRvizFile)

		self.__init()
		self.setFixedSize(400, 320)

	def __init(self):
		self.topLayout = QGridLayout()
		self.topLayout.addWidget(self.labelLaunchFilePath, 0, 0)
		self.topLayout.addWidget(self.labelRvizFilePath, 1, 0)
		self.topLayout.addWidget(self.labelNode, 2, 0)
		self.topLayout.addWidget(self.launchFilePath, 0, 1)
		self.topLayout.addWidget(self.rvizFilePath, 1, 1)
		self.topLayout.addWidget(self.node, 2, 1)
		self.topLayout.addWidget(self.browseButton, 0, 2)
		self.topLayout.addWidget(self.rvizBrowseButton, 1, 2)

		self.setLayout(self.topLayout)

	def __browseLaunchFile(self):
		if self.parent.parent.historypath == None:
			openpath = os.path.expandvars('$HOME')
		else:
			if os.path.isfile(self.parent.parent.historypath):
				openpath = os.path.dirname(self.parent.parent.historypath)
			elif os.path.isdir(self.parent.parent.historypath):
				openpath = self.parent.parent.historypath
		fileName = QFileDialog.getOpenFileName(self, "Open Launch File", openpath, "Launch Files (*.launch *.LAUNCH)")
		if str(fileName[0]) != '':
			self.parent.parent.historypath = str(fileName[0])
			self.launchFilePath.setText(str(fileName[0]))
			if self.launchFilePath.text() != '' and self.rvizFilePath.text() != '' and self.node.text() != '':
				self.parent.saveButton.setDisabled(False)
			else:
				self.parent.saveButton.setDisabled(True)

	def __browseRvizFile(self):
		if self.parent.parent.historypath == None:
			openpath = os.path.expandvars('$HOME')
		else:
			if os.path.isfile(self.parent.parent.historypath):
				openpath = os.path.dirname(self.parent.parent.historypath)
			elif os.path.isdir(self.parent.parent.historypath):
				openpath = self.parent.parent.historypath
		fileName = QFileDialog.getOpenFileName(self, "Open Rviz File", openpath, "Rviz Files (*.rviz *.RVIZ)")
		if str(fileName[0]) != '':
			self.parent.parent.historypath = str(fileName[0])
			self.rvizFilePath.setText(str(fileName[0]))
			if self.launchFilePath.text() != '' and self.rvizFilePath.text() != '' and self.node.text() != '':
				self.parent.saveButton.setDisabled(False)
			else:
				self.parent.saveButton.setDisabled(True)

	def __onChanged(self):
		if self.launchFilePath.text() != '' and self.rvizFilePath.text() != '' and self.node.text() != '':
			self.parent.saveButton.setDisabled(False)
		else:
			self.parent.saveButton.setDisabled(True)

class AdvancedTabWidget(QDialog):
	"""docstring for AdvancedTabWidget"""
	def __init__(self, parent, config):
		super(AdvancedTabWidget, self).__init__(parent)
		self.config = config
		self.parent = parent

		self.enableMapCheckBox = QCheckBox('Enable Map', self)
		self.enableMap = 0
		self.enableMapCheckBox.stateChanged.connect(self.__enableMap)

		self.labelMapFilePath = QLabel('Map file:')
		self.mapFilePath = QLineEdit()
		self.browseButton = QPushButton('&Browse')

		if self.enableMapCheckBox.isChecked() == False:
			self.enableMap = 0
			self.labelMapFilePath.setDisabled(True)
			self.mapFilePath.setDisabled(True)
			self.browseButton.setDisabled(True)
		else:
			self.enableMap = 1
			self.labelMapFilePath.setDisabled(False)
			self.mapFilePath.setDisabled(False)
			self.browseButton.setDisabled(False)

		self.connect(self.browseButton, SIGNAL("clicked()"), self.__browseMapFile)

		self.__init()
		self.setFixedSize(400, 320)

	def __init(self):
		self.topLayout = QGridLayout()
		self.topLayout.addWidget(self.enableMapCheckBox, 0, 0)
		self.topLayout.addWidget(self.labelMapFilePath, 1, 0)
		self.topLayout.addWidget(self.mapFilePath, 1, 1)
		self.topLayout.addWidget(self.browseButton, 1, 2)

		self.setLayout(self.topLayout)

	def __enableMap(self):
		if self.enableMapCheckBox.isChecked() == False:
			self.enableMap = 0
			self.labelMapFilePath.setDisabled(True)
			self.mapFilePath.setDisabled(True)
			self.browseButton.setDisabled(True)
		else:
			self.enableMap = 1
			self.labelMapFilePath.setDisabled(False)
			self.mapFilePath.setDisabled(False)
			self.browseButton.setDisabled(False)

	def __browseMapFile(self):
		if self.parent.parent.historypath == None:
			openpath = os.path.expandvars('$HOME')
		else:
			if os.path.isfile(self.parent.parent.historypath):
				openpath = os.path.dirname(self.parent.parent.historypath)
			elif os.path.isdir(self.parent.parent.historypath):
				openpath = self.parent.parent.historypath
		fileName = QFileDialog.getOpenFileName(self, "Open Map File", openpath, "Yaml Files (*.yaml *.YAML)")
		if str(fileName[0]) != '':
			self.parent.parent.historypath = str(fileName[0])
			self.mapFilePath.setText(str(fileName[0]))
