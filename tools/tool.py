#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
import locale
from PyQt4.QtCore import *
from PyQt4.QtGui import *

code = QTextCodec.codecForName(locale.getpreferredencoding())
QTextCodec.setCodecForLocale(code)									# 设置程序能够正确读取到的本地文件的编码方式
QTextCodec.setCodecForTr(code)										# 使用设定的code编码来解析字符串方法
QTextCodec.setCodecForCStrings(code)

BASHRCFILE = os.path.join(os.path.expandvars('$HOME'), '.bashrc')
HOSTSFILE = os.path.join('/', 'etc', 'hosts')

class MainWindow( QWidget ):
	"""docstring for MainWindow"""
	def __init__(self):
		super(MainWindow, self).__init__()
		self.tab1 = BashrcFile(self)
		# self.tab2 = HostsFile(self)

		self.baseTabWidget = QTabWidget()
		self.baseTabWidget.setTabPosition(QTabWidget.North)

		self.baseTabWidget.addTab(self.tab1, self.tr('修改.bashrc文件'))
		# self.baseTabWidget.addTab(self.tab2, self.tr('修改host文件'))

		self.setFixedSize(640, 480)
		self.setWindowTitle('一键配置ROS主从机工具')
		self.setWindowModality(Qt.ApplicationModal)

		# self.tab2.setEnabled(False)

		self.init()
		self.show()
	
	def init(self):

		self.mainLayout = QVBoxLayout()
		self.mainLayout.addWidget(self.baseTabWidget)

		self.setLayout(self.mainLayout)


class BashrcFile(QDialog):
	"""docstring for BashrcFile"""
	def __init__(self, parent):
		super(BashrcFile, self).__init__()
		regExp = QRegExp("^((?:(2[0-4]\d)|(25[0-5])|([01]?\d\d?))\.){3}(?:(2[0-4]\d)|(255[0-5])|([01]?\d\d?))$")
		self.parent = parent
		self.nameLabel = QLabel('ROS主机名: ')
		self.name = QLineEdit()
		self.name.textChanged[str].connect(self.onTextChanged)
		self.recoverButton = QPushButton('恢 复')
		self.saveButton = QPushButton('保 存')
		self.ipLabel = QLabel('主机IP地址: ')
		self.ip = QLineEdit()
		self.ip.textChanged[str].connect(self.onTextChanged)
		self.ip.setValidator(QRegExpValidator(regExp,self))
		self.lines = self.getFileLines()
		self.text = QTextEdit()
		self.text.setReadOnly(True)

		if os.path.exists(BASHRCFILE + '.bak') != True:
			self.recoverButton.setEnabled(False)
		else:
			self.recoverButton.setEnabled(True)

		self.saveButton.setEnabled(False)
		self.connect(self.saveButton, SIGNAL("clicked()"), self.saveFile)
		self.connect(self.recoverButton, SIGNAL("clicked()"), self.recoverFile)
		self.__init()
		# self.setFileLines(BASHRCFILE + '.tmp', lines)
		temp = ''
		for line in self.lines:
			temp += str(line)
		self.text.setText(temp)
		self.text.scrollContentsBy(1, 100)
		
	def __init(self):
		self.widget = QHBoxLayout()
		self.leftLayout = QVBoxLayout()
		self.rightLayout = QVBoxLayout()
		self.onTopLayout = QHBoxLayout()
		self.onBottomLayout = QHBoxLayout()
		self.onTopLayout.addWidget(self.nameLabel)
		self.onTopLayout.addWidget(self.name)
		self.onBottomLayout.addWidget(self.ipLabel)
		# self.onBottomLayout.addStretch()
		self.onBottomLayout.addWidget(self.ip)

		self.leftLayout.addLayout(self.onTopLayout)
		self.leftLayout.addLayout(self.onBottomLayout)
		self.rightLayout.addWidget(self.saveButton)
		self.rightLayout.addWidget(self.recoverButton)

		self.widget.addLayout(self.leftLayout)
		self.widget.addLayout(self.rightLayout)

		self.bottomLayout = QHBoxLayout()
		self.bottomLayout.addWidget(self.text)

		self.mainLayout = QVBoxLayout()
		self.mainLayout.addLayout(self.widget)
		self.mainLayout.addLayout(self.bottomLayout)

		self.setLayout(self.mainLayout)

	def getFileLines(self):
		with open(BASHRCFILE, 'rb') as f:
			return f.readlines()

	def setFileLines(self, filename, buf):
		with open(filename, 'wb') as f:
			f.writelines(buf)

	def onTextChanged(self):
		if self.name.text() != '' and self.ip.text() != '':
			self.saveButton.setDisabled(False)
		else:
			self.saveButton.setDisabled(True)

	def __saveTempFile(self):
		tmpFileName = BASHRCFILE + '.bak'
		self.setFileLines(tmpFileName, self.lines)

	def recoverFile(self):
		tmpFileName = BASHRCFILE + '.bak'
		with open(tmpFileName, 'rb') as f:
			lines = f.readlines()
		self.setFileLines(BASHRCFILE, lines)
		os.remove(tmpFileName)
		temp = ''
		for line in lines:
			temp += str(line)
		self.text.setText(temp)
		os.system('. ' + BASHRCFILE)
		self.recoverButton.setEnabled(False)

	def saveFile(self):
		name = str(self.name.text())
		ip = str(self.ip.text())
		lines = self.lines
		nameline = 'export ROS_HOSTNAME=' + name + '\n'
		ipline = 'export ROS_MASTER_URI=http://' + ip + ':11311\n'
		if os.path.exists(BASHRCFILE + '.bak') != True:
			self.__saveTempFile()
		if 'export ROS_HOSTNAME=' in lines[-2]:
			lines[-2] = nameline
		else:
			lines.append(nameline)
		if 'export ROS_MASTER_URI=http://' in lines[-1]:
			lines[-1] = ipline
		else:
			lines.append(ipline)
		temp = ''
		for line in lines:
			temp += str(line)
		self.text.setText(temp)
		self.text.scrollContentsBy(1, 100)
		self.setFileLines(BASHRCFILE, lines)
		os.system('. ' + BASHRCFILE)
		self.recoverButton.setEnabled(True)

class HostsFile(QDialog):
	"""docstring for HostsFile"""
	def __init__(self, parent):
		super(HostsFile, self).__init__()
		regExp = QRegExp("^((?:(2[0-4]\d)|(25[0-5])|([01]?\d\d?))\.){3}(?:(2[0-4]\d)|(255[0-5])|([01]?\d\d?))$")
		self.parent = parent
		self.localNameLabel = QLabel('ROS主机名: ')
		self.localName = QLineEdit()
		self.hostNameLabel = QLabel('ROS从机名: ')
		self.hostName = QLineEdit()
		self.recoverButton = QPushButton(u'恢 复')
		self.saveButton = QPushButton(u'保 存')
		self.localipLabel = QLabel('主机IP地址: ')
		self.localip = QLineEdit()
		self.localip.setValidator(QRegExpValidator(regExp,self))
		self.hostipLabel = QLabel('从机IP地址: ')
		self.hostip = QLineEdit()
		self.hostip.setValidator(QRegExpValidator(regExp,self))
		self.lines = self.getFileLines()
		self.text = QTextEdit()
		self.button = QPushButton()
		self.text.setReadOnly(True)
		self.__init()
		# self.setFileLines(BASHRCFILE + '.tmp', lines)
		temp = ''
		for line in self.lines:
			temp += str(line)
		self.text.setText(temp)
		
	def __init(self):
		self.widget = QHBoxLayout()
		self.leftLayout = QVBoxLayout()
		self.rightLayout = QVBoxLayout()
		self.onTopLayout = QHBoxLayout()
		self.onBottomLayout = QHBoxLayout()
		self.onTopLayout.addWidget(self.localNameLabel)
		self.onTopLayout.addWidget(self.localName)
		self.onTopLayout.addWidget(self.hostNameLabel)
		self.onTopLayout.addWidget(self.hostName)
		self.onBottomLayout.addWidget(self.localipLabel)
		self.onBottomLayout.addWidget(self.localip)
		self.onBottomLayout.addWidget(self.hostipLabel)
		self.onBottomLayout.addWidget(self.hostip)

		self.leftLayout.addLayout(self.onTopLayout)
		self.leftLayout.addLayout(self.onBottomLayout)
		self.rightLayout.addWidget(self.saveButton)
		self.rightLayout.addWidget(self.recoverButton)

		self.widget.addLayout(self.leftLayout)
		self.widget.addLayout(self.rightLayout)

		self.bottomLayout = QHBoxLayout()
		self.bottomLayout.addWidget(self.text)

		self.mainLayout = QVBoxLayout()
		self.mainLayout.addLayout(self.widget)
		self.mainLayout.addLayout(self.bottomLayout)

		self.setLayout(self.mainLayout)

	def getFileLines(self):
		with open(HOSTSFILE, 'rb') as f:
			return f.readlines()

	def setFileLines(self, filename, buf):
		with open(filename, 'wb') as f:
			f.writelines(buf)

	def onTextChanged(self):
		pass	

def main(*args):

	app = QApplication(sys.argv)
	MWindow = MainWindow()
	sys.exit(app.exec_())

if __name__ == '__main__':
	main()
