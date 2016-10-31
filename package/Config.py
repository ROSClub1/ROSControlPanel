#!/usr/bin/env python
# -*- coding: utf-8 -*-

import ConfigParser
import os.path
import io

"""
Read and save config files

File format Example:

[filepath]
launchpath = ''
rvizpath = ''
mapfilepath = ''
[setting]
node = ''
name = ''
enablerviz = 0
enablecontroller = 0
enablemap = 0
"""

class Config():
	"""docstring for Config"""
	def __init__(self, configpath):
		self.__configpath = configpath
		self.__config = ConfigParser.RawConfigParser()
		if os.path.isfile(self.__configpath):
			self.__config.readfp(open(self.__configpath))
		else:
			self.initConfigFile(self.__configpath)
			self.__config.readfp(open(self.__configpath))

		# initConfigFile(self, **args)
	def initConfigFile(self, configFilePath, filepath = '', rvizpath = '', launchpath = '', node = '', name = '', mapFilePath = '', enableMap = 0, enableRviz = 0, enableController = 0):
		self.__config.add_section('filepath')
		self.__config.set('filepath', 'launchpath', launchpath)
		self.__config.set('filepath', 'rvizpath', rvizpath)
		self.__config.set('filepath', 'mapfilepath', mapFilePath)
		self.__config.add_section('setting')
		self.__config.set('setting', 'node', node)
		self.__config.set('setting', 'name', name)
		self.__config.set('setting', 'enableRviz', enableRviz)
		self.__config.set('setting', 'enableController', enableController)
		self.__config.set('setting', 'enableMap', enableMap)

		with open(configFilePath, 'wb') as configfile:
			self.__config.write(configfile)
		self.__config.readfp(open(self.__configpath))

		# saveSetting(self, **args)
	def saveSetting(self, rvizpath, launchpath, node, name, mapFilePath, enableMap, enableRviz, enableController, configFilePath):
		self.__config.set('filepath', 'launchpath', launchpath)
		self.__config.set('filepath', 'rvizpath', rvizpath)
		self.__config.set('filepath', 'mapfilepath', mapFilePath)
		self.__config.set('setting', 'node', node)
		self.__config.set('setting', 'name', name)
		self.__config.set('setting', 'enableRviz', enableRviz)
		self.__config.set('setting', 'enableController', enableController)
		self.__config.set('setting', 'enableMap', enableMap)

		with open(configFilePath, 'wb') as configfile:
			self.__config.write(configfile)
		self.__config.readfp(open(self.__configpath))

	def getValue(self, section, key):
		return(self.__config.get(section, key))

	def setValue(self, section, key, value):
		self.__config.set(section, key, value)