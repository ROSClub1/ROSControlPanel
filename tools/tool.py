#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys, pexpect
import locale
from PyQt4.QtCore import *
from PyQt4.QtGui import *

code = QTextCodec.codecForName(locale.getpreferredencoding())
QTextCodec.setCodecForLocale(code)									# 设置程序能够正确读取到的本地文件的编码方式
QTextCodec.setCodecForTr(code)										# 使用设定的code编码来解析字符串方法
QTextCodec.setCodecForCStrings(code)

BASHRCFILE = os.path.join(os.path.expandvars('$HOME'), '.bashrc')
TMPFILE = os.path.join('/', 'tmp', 'bashrc.tmp')

PROMPT = ['# ', '>>> ', '> ', '\$ ']

def sendCommand(child, cmd):
	child.sendline(cmd)
	child.expect(PROMPT)

def connect(user, host, password):
	ssh_newkey = 'Are you sure you want to continue connecting'
	connStr = 'ssh ' + str(user) + '@' + str(host)
	child = pexpect.spawn(connStr)
	try:
		ret = child.expect([pexpect.TIMEOUT, ssh_newkey, '[P|p]assword:'])
		if ret == 0:
			return -1
		if ret == 1:
			child.sendline('yes')
			ret = child.expect([pexpect.TIMEOUT, '[P|p]assword:'])
			if ret == 0:
				return -1
		child.sendline(str(password))
		child.expect(PROMPT)
		return child
	except pexpect.TIMEOUT:
		return -1
	except pexpect.EOF:
		return -1

def getFile(user, host, password):
	ssh_newkey = 'Are you sure you want to continue connecting'
	connStr = 'scp ' + str(user) + '@' + str(host) + ':~/.bashrc ' + TMPFILE
	child = pexpect.spawn(connStr)
	
	try:
		ret = child.expect([pexpect.TIMEOUT, ssh_newkey, '[P|p]assword:'])
		if ret == 0:
			return -1
		if ret == 1:
			child.sendline('yes')
			ret = child.expect([pexpect.TIMEOUT, '[P|p]assword:'])
			if ret == 0:
				return -1
		child.sendline(str(password))
		child.interact()
		return 0
	except pexpect.TIMEOUT:
		return -1
	except pexpect.EOF:
		return -1

def putFile(user, host, password):
	ssh_newkey = 'Are you sure you want to continue connecting'
	connStr = 'scp ' + TMPFILE + ' ' + str(user) + '@' + str(host) + ':~/.bashrc'
	child = pexpect.spawn(connStr)
	
	try:
		ret = child.expect([pexpect.TIMEOUT, ssh_newkey, '[P|p]assword:'])
		if ret == 0:
			return -1
		if ret == 1:
			child.sendline('yes')
			ret = child.expect([pexpect.TIMEOUT, '[P|p]assword:'])
			if ret == 0:
				return -1
		child.sendline(str(password))
		child.interact()
		return 0
	except pexpect.TIMEOUT:
		return -1
	except pexpect.EOF:
		return -1

def getLocalIP(ifname): 
	import socket, fcntl, struct 
	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
	inet = fcntl.ioctl(s.fileno(), 0x8915, struct.pack('256s', ifname[:15])) 
	ret = socket.inet_ntoa(inet[20:24]) 
	return ret 

class MainWindow( QWidget ):
	"""docstring for MainWindow"""
	def __init__(self):
		super(MainWindow, self).__init__()
		self.master = None
		self.tab = MasterBashrcFile(self)

		# self.baseTabWidget = QTabWidget()
		# self.baseTabWidget.setTabPosition(QTabWidget.North)

		# self.baseTabWidget.addTab(self.tab, self.tr('配   置'))

		self.setFixedSize(640, 480)
		self.setWindowTitle('一键配置ROS主从机工具')
		self.setWindowModality(Qt.ApplicationModal)

		self.init()
		self.show()
	
	def init(self):

		self.mainLayout = QVBoxLayout()
		self.mainLayout.addWidget(self.tab)

		self.setLayout(self.mainLayout)

class MasterBashrcFile(QDialog):
	"""docstring for MasterBashrcFile"""
	def __init__(self, parent):
		super(MasterBashrcFile, self).__init__()
		regExp = QRegExp("^((?:(2[0-4]\d)|(25[0-5])|([01]?\d\d?))\.){3}(?:(2[0-4]\d)|(255[0-5])|([01]?\d\d?))$")
		self.parent = parent
		self.checkBox = QCheckBox('主机为本机')
		self.checkBox.toggle()
		self.parent.master = 1
		self.checkBox.stateChanged.connect(self.__settingLogin)
		self.hostIPLabel = QLabel('从机IP地址: ')
		self.hostIP = QLineEdit()
		self.UserNameLabel = QLabel('用户名: ')
		self.UserName = QLineEdit()
		self.PassWordLabel = QLabel('密     码: ')
		self.PassWord = QLineEdit()
		self.PassWord.setEchoMode(QLineEdit.Password)
		self.UserName.textChanged[str].connect(self.onTextChanged)
		self.PassWord.textChanged[str].connect(self.onTextChanged)
		self.hostIP.textChanged[str].connect(self.onTextChanged)
		self.recoverButton = QPushButton('恢 复')
		self.saveButton = QPushButton('保 存')
		self.masterIPLabel = QLabel('主机IP地址: ')
		self.masterIP = QLineEdit()
		self.masterIP.setText(getLocalIP('eth0'))
		self.masterIP.textChanged[str].connect(self.onTextChanged)
		self.masterIP.setValidator(QRegExpValidator(regExp,self))

		self.lines = None

		self.saveButton.setEnabled(False)
		self.recoverButton.setEnabled(False)
		self.connect(self.saveButton, SIGNAL("clicked()"), self.saveFile)
		self.connect(self.recoverButton, SIGNAL("clicked()"), self.recoverFile)
		self.__init()

	def warning(self):
		QMessageBox.critical(self, "错误", self.tr("远程设备无法连接!"))

	def information(self):
		QMessageBox.information(self, "信息", self.tr("主从机配置已完成"))
		
	def __init(self):
		self.frame = QVBoxLayout()
		self.widget = QHBoxLayout()
		self.leftLayout = QVBoxLayout()
		self.rightLayout = QVBoxLayout()
		self.onTopLayout = QHBoxLayout()
		self.loginGroup = QGroupBox()
		self.loginGroup.setTitle("远程设备登录")
		self.loginGroup.setStyleSheet("QGroupBox{background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1,stop:0 #E0E0E0, stop: 1 #FFFFFF);border:2px solid gray;border-radius:5px;margin-top:1ex;}QGroupBox::title {subcontrol-origin: margin;subcontrol-position: top center;padding:03px;}")
		self.loginLayout = QVBoxLayout(self.loginGroup)
		self.UserNameLayout = QHBoxLayout()
		self.PassWordLayout = QHBoxLayout()
		self.onBottomLayout = QHBoxLayout()
		self.onTopLayout.addWidget(self.hostIPLabel)
		self.onTopLayout.addWidget(self.hostIP)
		self.onBottomLayout.addWidget(self.masterIPLabel)
		self.onBottomLayout.addWidget(self.masterIP)
		self.UserNameLayout.addWidget(self.UserNameLabel)
		self.UserNameLayout.addWidget(self.UserName)
		self.PassWordLayout.addWidget(self.PassWordLabel)
		self.PassWordLayout.addWidget(self.PassWord)

		self.leftLayout.addLayout(self.onTopLayout)
		self.leftLayout.addLayout(self.onBottomLayout)
		self.loginLayout.addLayout(self.UserNameLayout)
		self.loginLayout.addLayout(self.PassWordLayout)
		self.rightLayout.addWidget(self.saveButton)
		self.rightLayout.addWidget(self.recoverButton)

		self.widget.addLayout(self.leftLayout)
		self.widget.addWidget(self.loginGroup)
		self.widget.addLayout(self.rightLayout)

		self.frame.addWidget(self.checkBox)
		self.frame.addLayout(self.widget)

		self.bottomLayout = QHBoxLayout()
		self.tab1 = QTextEdit()
		self.tab2 = QTextEdit()
		self.tab1.setReadOnly(True)
		self.tab2.setReadOnly(True)
		self.baseTabWidget = QTabWidget()
		self.baseTabWidget.setTabPosition(QTabWidget.North)

		self.baseTabWidget.addTab(self.tab1, self.tr('主机配置文件内容'))
		self.baseTabWidget.addTab(self.tab2, self.tr('从机配置文件内容'))
		self.bottomLayout.addWidget(self.baseTabWidget)

		self.mainLayout = QVBoxLayout()
		self.mainLayout.addLayout(self.frame)
		self.mainLayout.addLayout(self.bottomLayout)

		self.setLayout(self.mainLayout)

	def __settingLogin(self):
		if self.parent.master == None:
			self.parent.master = 1
			self.masterIP.setText(getLocalIP('eth0'))
			self.hostIP.setText('')
		elif self.parent.master == 1:
			self.parent.master = 2
			self.hostIP.setText(getLocalIP('eth0'))
			self.masterIP.setText('')
		elif self.parent.master == 2:
			self.parent.master = 1
			self.masterIP.setText(getLocalIP('eth0'))
			self.hostIP.setText('')

	def getFileLines(self, filename):
		with open(filename, 'rb') as f:
			return f.readlines()

	def setFileLines(self, filename, buf):
		with open(filename, 'wb') as f:
			f.writelines(buf)

	def onTextChanged(self):
		if self.hostIP.text() != '' and self.masterIP.text() != '':
			self.saveButton.setDisabled(False)
		else:
			self.saveButton.setDisabled(True)

	# def __saveTempFile(self):
	# 	tmpFileName = BASHRCFILE + '.bak'
	# 	self.setFileLines(tmpFileName, self.lines)

	def recoverFile(self):
		tmpFileName = BASHRCFILE + '.bak'
		with open(tmpFileName, 'rb') as f:
			lines = f.readlines()
		self.setFileLines(BASHRCFILE, lines)
		os.remove(tmpFileName)
		temp = ''
		for line in lines:
			temp += str(line)
		self.tab1.setText(temp)
		os.system('. ' + BASHRCFILE)
		self.recoverButton.setEnabled(False)

	def saveFile(self):
		if self.checkBox.isChecked():

			masterFileLines = self.getFileLines(BASHRCFILE)
			masterIP = self.masterIP.text()
			hostIP = self.hostIP.text()
			f = getFile(unicode(QString(self.UserName.text()).toUtf8(), 'utf-8', 'ignore'), hostIP, unicode(QString(self.PassWord.text()).toUtf8(), 'utf-8', 'ignore'))
			if f == -1:
				self.warning()
				return
			else:
				hostFileLines = self.getFileLines(TMPFILE)
				masterFileLine = 'export ROS_IP=' + masterIP + '	#rosIP\n'
				hostFileURILine = 'export ROS_MASTER_URI=http://' + masterIP + ':11311		#ros Master URI\n'
				hostFileIPLine = 'export ROS_IP=' + hostIP + '	#rosIP\n'
				masterFileFindLine = False
				hostFileFindURILine = False
				hostFileFindIPLine = False

				for i in range(0, len(masterFileLines) - 1):
					if '#rosIP' in masterFileLines[i]:
						masterFileLines[i] = masterFileLine
						masterFileFindLine = True

				if masterFileFindLine == False:
					masterFileLines.append(masterFileLine)
					masterFileFindLine = True

				for i in range(0, len(hostFileLines) - 1):
					if '#rosIP' in hostFileLines[i]:
						hostFileLines[i] = hostFileIPLine
						hostFileFindIPLine = True
					if '#ros Master URI' in hostFileLines[i]:
						hostFileLines[i] = hostFileURILine
						hostFileFindURILine = True

				if hostFileFindIPLine == False:
					hostFileLines.append(hostFileIPLine)
					hostFileFindIPLine = True
				if hostFileFindURILine == False:
					hostFileLines.append(hostFileURILine)
					hostFileFindURILine = True

				temp1 = ''
				for line in masterFileLines:
					temp1 += str(line)
				temp2 = ''
				for line in hostFileLines:
					temp2 += str(line)
				self.tab1.setText(temp1)
				self.tab2.setText(temp2)
				self.setFileLines(BASHRCFILE, masterFileLines)
				self.setFileLines(TMPFILE, hostFileLines)
				f = putFile(unicode(QString(self.UserName.text()).toUtf8(), 'utf-8', 'ignore'), hostIP, unicode(QString(self.PassWord.text()).toUtf8(), 'utf-8', 'ignore'))
				if f == -1:
					self.warning()
					return
				else:
					child = connect(unicode(QString(self.UserName.text()).toUtf8(), 'utf-8', 'ignore'), hostIP, unicode(QString(self.PassWord.text()).toUtf8(), 'utf-8', 'ignore'))
					if child == -1:
						self.warning()
						return
					else:
						sendCommand(child, 'source ~/.bashrc')
						child.sendline('exit')
						os.system('. ' + BASHRCFILE)
			self.information()
		else:
			hostFileLines = self.getFileLines(BASHRCFILE)
			masterIP = self.masterIP.text()
			hostIP = self.hostIP.text()
			f = getFile(unicode(QString(self.UserName.text()).toUtf8(), 'utf-8', 'ignore'), hostIP, unicode(QString(self.PassWord.text()).toUtf8(), 'utf-8', 'ignore'))
			if f == -1:
				self.warning()
				return
			else:
				masterFileLines = self.getFileLines(TMPFILE)
				masterFileLine = 'export ROS_IP=' + masterIP + '	#rosIP\n'
				hostFileURILine = 'export ROS_MASTER_URI=http://' + masterIP + ':11311		#ros Master URI\n'
				hostFileIPLine = 'export ROS_IP=' + hostIP + '	#rosIP\n'
				masterFileFindLine = False
				hostFileFindURILine = False
				hostFileFindIPLine = False

				for i in range(0, len(masterFileLines) - 1):
					if '#rosIP' in masterFileLines[i]:
						masterFileLines[i] = masterFileLine
						masterFileFindLine = True

				if masterFileFindLine == False:
					masterFileLines.append(masterFileLine)
					masterFileFindLine = True

				for i in range(0, len(hostFileLines) - 1):
					if '#rosIP' in hostFileLines[i]:
						hostFileLines[i] = hostFileIPLine
						hostFileFindIPLine = True
					if '#ros Master URI' in hostFileLines[i]:
						hostFileLines[i] = hostFileURILine
						hostFileFindURILine = True

				if hostFileFindIPLine == False:
					hostFileLines.append(hostFileIPLine)
					hostFileFindIPLine = True
				if hostFileFindURILine == False:
					hostFileLines.append(hostFileURILine)
					hostFileFindURILine = True

				temp1 = ''
				for line in masterFileLines:
					temp1 += str(line)
				temp2 = ''
				for line in hostFileLines:
					temp2 += str(line)
				self.tab1.setText(temp1)
				self.tab2.setText(temp2)
				self.setFileLines(BASHRCFILE, masterFileLines)
				self.setFileLines(TMPFILE, hostFileLines)
				f = putFile(unicode(QString(self.UserName.text()).toUtf8(), 'utf-8', 'ignore'), hostIP, unicode(QString(self.PassWord.text()).toUtf8(), 'utf-8', 'ignore'))
				if f == -1:
					self.warning()
					return
				else:
					child = connect(unicode(QString(self.UserName.text()).toUtf8(), 'utf-8', 'ignore'), hostIP, unicode(QString(self.PassWord.text()).toUtf8(), 'utf-8', 'ignore'))
					if child == -1:
						self.warning()
						return
					else:
						sendCommand(child, 'source ~/.bashrc')
						child.sendline('exit')
						os.system('. ' + BASHRCFILE)
		# self.recoverButton.setEnabled(True)
			self.information()

def main(*args):

	app = QApplication(sys.argv)
	MWindow = MainWindow()
	sys.exit(app.exec_())

if __name__ == '__main__':
	main()
