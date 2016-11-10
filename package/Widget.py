#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from Control import *
import rviz

# __path__ = os.path.split(os.path.realpath(__file__))[0]

class MainWidget( QWidget ):
	"""docstring for MainWindow"""
	def __init__(self, parent, path, rvizPath, node):
		super(MainWidget, self).__init__(parent)
		self.parent = parent
		self.path = path
		self.node = node
		self.rvizPath = rvizPath
		self.btn1 = QPushButton("modal dialog", clicked = self.on_btn1_clicked)
		self.btn1.setMaximumWidth(87)
		self.btn2 = QPushButton("modal dialog", clicked = self.on_btn2_clicked)
		self.btn2.setMaximumWidth(87)
		self.btn3 = QPushButton("modal dialog", clicked = self.on_btn3_clicked)
		self.btn3.setMaximumWidth(87)
		self.btn1.setDisabled(True)
		self.btn2.setDisabled(True)
		self.btn3.setDisabled(True)
		self.setWindowTitle('ROS Control Panel')
		# if not self.isFullScreen():
		# 	self.showMaximized()
		self.resize(1280, 720)
		self.setWindowIcon(QIcon(''.join([self.path, '/source/icon.jpg'])))
		self.ros = Control(self.node)

		self.InitRviz(self.rvizPath)

	def on_btn1_clicked(self, *args):
		pass

	def on_btn2_clicked(self, *args):
		d = QDialog(self)
		d.setWindowModality(Qt.ApplicationModal)
		d.show()
		d.raise_()
		d.activateWindow()

	def on_btn3_clicked(self, *args):
		pass

	def changeLabel_1_Value(self, value):

		# 通过滑块的值，我们可以改变标签的文本。
		self.parent.l1.setText('Angular Speed: ' + str(value / 10) + ' rad/s')
		self.ros.Turn(value / 10)

	def changeLabel_2_Value(self, value):

		# 通过滑块的值，我们可以改变标签的文本。
		self.parent.l2.setText('Linear Speed: ' + str(value / 10) + ' m/s')
		self.ros.Forward(value / 10)

	def InitRviz(self, rvizPath):
		self.l3 = QLabel('Angular Speed')
		self.l3.setAlignment(Qt.AlignCenter)
		self.l4 = QLabel('Linear Speed')
		self.l4.setAlignment(Qt.AlignCenter)
		self.frame = rviz.VisualizationFrame()
		self.frame.setSplashPath( "" )
		self.frame.initialize()

		reader = rviz.YamlConfigReader()
		config = rviz.Config()
		reader.readFile( config, rvizPath )

		self.frame.load( config )

		self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )

		self.frame.setMenuBar( None )
		self.frame.setStatusBar( None )
		self.frame.setHideButtonVisibility( False )

		self.manager = self.frame.getManager()

		self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )

		rvizLayout = QVBoxLayout()
		rvizLayout.addWidget( self.frame )

		self.angularDial = QDial(self)
		self.angularDial.setGeometry(QRect(10, 20, 311, 351))
		self.angularDial.setMinimum(-10)
		self.angularDial.setMaximum(10)
		self.angularDial.setTracking(True)
		
		# initial dial pointer position
		dial_pos = 50
		self.angularDial.setSliderPosition(dial_pos)
		self.angularDial.setWrapping(False)
		self.angularDial.setNotchTarget(0.1)
		self.angularDial.setNotchesVisible(True)
		self.angularDial.setValue(0)
		self.angularDial.valueChanged.connect(self.changeLabel_1_Value)

		self.linearSpeedDial = QDial(self)
		self.linearSpeedDial.setGeometry(QRect(10, 20, 30, 30))
		self.linearSpeedDial.setMinimum(-10)
		self.linearSpeedDial.setMaximum(10)
		self.linearSpeedDial.setTracking(True)
		
		# initial dial pointer position
		self.linearSpeedDial.setSliderPosition(dial_pos)
		self.linearSpeedDial.setWrapping(False)
		self.linearSpeedDial.setNotchTarget(0.1)
		self.linearSpeedDial.setNotchesVisible(True)
		self.linearSpeedDial.setValue(0)
		self.linearSpeedDial.valueChanged.connect(self.changeLabel_2_Value)

		controlPanelLayout = QGridLayout()
		controlPanelLayout.setSpacing(6)
		controlPanelLayout.addWidget( self.l3, 0, 6, 1, 3)
		controlPanelLayout.addWidget( self.l4, 0, 9, 1, 3)
		controlPanelLayout.addWidget( self.btn1, 0, 1, 1, 1)
		controlPanelLayout.addWidget( self.btn2, 1, 3, 1, 1)
		controlPanelLayout.addWidget( self.btn3, 2, 5, 1, 1)
		self.parent.statusBar().addPermanentWidget(self.parent.l1)
		self.parent.statusBar().addPermanentWidget(self.parent.l2)
		
		controlPanelLayout.addWidget( self.angularDial, 1, 6, 3, 3)
		controlPanelLayout.addWidget( self.linearSpeedDial, 1, 9, 3, 3)

		mainLayout = QVBoxLayout()
		mainLayout.addLayout(rvizLayout)
		mainLayout.addLayout( controlPanelLayout )

		self.setLayout( mainLayout )
