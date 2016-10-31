#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib

import rospy
from geometry_msgs.msg import Twist
import rviz


class Control(object):
	"""docstring for ClassName"""
	def __init__(self, node = '/cmd_vel'):
		self.n = node
		self.pub = rospy.Publisher(self.n, Twist, queue_size = 10)
		rospy.init_node('ControlPanel')						# Give the node a name
		rospy.on_shutdown(self.shutdown)					# Set rospy to execute a shutdown function when terminating the script
		self.rate = rospy.Rate(rospy.get_param('~hz', 50))
		self.cmd = Twist()

	def SetLinearSpeed(self, x, y, z):
		self.cmd.linear.x = x
		self.cmd.linear.y = y
		self.cmd.linear.z = z

	def SetAngularSpeed(self, x, y, z):
		self.cmd.angular.x = x
		self.cmd.angular.y = y
		self.cmd.angular.z = z

	def Forward(self, speed):
		self.SetLinearSpeed(speed, 0, 0)					# Linear speed unit is m/s
		self.pub.publish(self.cmd)
		self.rate.sleep()

	def Turn(self, speed):
		self.SetAngularSpeed(0, 0, speed)
		self.pub.publish(self.cmd)
		self.rate.sleep()

	def shutdown(self):										# Always stop the robot when shutting down the node
		rospy.loginfo("Stopping the robot...")
		self.pub.publish(Twist())
		rospy.sleep(1)