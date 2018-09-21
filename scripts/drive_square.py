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


class Neato(object):

	#Taken from Paul's neato code
	def __init__(self):
		rospy.init_node('drive_square')
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
		rospy.on_shutdown(self.shutdown_func)
		self.update_rate = rospy.Rate(10)
		self.timout = time.time()

	def shutdown_func(self):
		self.pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))

	def square(self, t):
		x=0
		y=0

		for z in range(1,4):
			start = time.time()
			print(start)
			while time.time()-start<4.2:
				self.pub.publish(Twist(Vector3(.3,0,0),Vector3(0,0,0)))
				time.sleep(1)
			self.pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))
			start = time.time()
			while time.time()-start<3: #CANNOT FIND A PROPER AMOUNT OF TIME TO TURN
				self.pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,.5)))
				time.sleep(1)
			self.pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))

	def run(self):
		while not rospy.is_shutdown():
			self.square(4)

			self.update_rate.sleep()


if __name__ == "__main__":
	neato1 = Neato()
	neato1.run()
