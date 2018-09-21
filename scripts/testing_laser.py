#!/usr/bin/env python

from __future__ import print_function, division
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
import tty
import select
import sys
import termios
import rospy
import time
import math


class Neato(object):
	def __init__(self):
		rospy.init_node('testing_laser')
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
		self.sub_laser = rospy.Subscriber('/stable_scan', LaserScan, self.laser_callback)
		self.update_rate = rospy.Rate(10)
		self.timout = time.time()
		self.laser_data = []

	
	def has_laser(self):
		for l in self.laser_data:
			if l > 0 and l < .5:
				return True
		return False

	def laser_callback(self, msg):
		self.laser_data = msg.ranges

	def run(self):
		time.sleep(4)
		"""
		for i in self.laser_data:
			print(i)
		"""
		while not rospy.is_shutdown():
			print(self.laser_data[94])

			if self.laser_data[180] <= 1 and  self.laser_data[180] != 0:
				start = time.time()
				while time.time()-start<2: 
					self.pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,.3)))
					time.sleep(1)
			else:
				self.pub.publish(Twist(Vector3(-.2,0,0),Vector3(0,0,0)))
			self.update_rate.sleep()








if __name__ == "__main__":
	neato1 = Neato()
	neato1.run()