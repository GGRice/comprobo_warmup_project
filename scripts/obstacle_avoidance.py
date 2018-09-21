#!/usr/bin/env python

from __future__ import print_function, division
from geometry_msgs.msg import Twist, Vector3
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from neato_node.msg import Bump
import termios
import select
import rospy
import time
import math
import sys
import tty


"""

This code allows the robot to autonomously maneuver around without hitting things

"""


class Neato(object):
	def __init__(self):
		#initializes rospy the publishers and subscribers, sets up the on shitdown, and initializes a lot of variabels
		rospy.init_node('obstacle_avoidance')
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
		self.sub_laser = rospy.Subscriber('/stable_scan', LaserScan, self.laser_callback)
		rospy.on_shutdown(self.shutdown_func)
		self.update_rate = rospy.Rate(10)
		self.timout = time.time()
		self.laser_data = []
		self.dist_from_obj = 0 #will be updated to save distance from an object


	def laser_callback(self, msg):
		self.laser_data = msg.ranges


	def shutdown_func(self):
		self.pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))


	#saw something straight ahead, avoid it
	def avoid(self):
		#turns left around an object if that space appears to be open
		#if there is something to the right and not to the left, it will turn towards the left
		if self.laser_data[94] <= 1 and (self.laser_data[270] >= 1 or self.laser_data[270]  == 0):
			self.turn_avoid(2, .3)

			self.dist_from_obj = self.laser_data[94]
			self.drive_object()

			self.turn_avoid(2, -.3)

		#turns right around an object if that space appears to be open
		#if there is something to the right and not to the left, it will turn towards the left		
		elif (self.laser_data[94] >= 1 or self.laser_data[270]  == 0) and self.laser_data[270] <= 1:
			self.turn_avoid(2, -.3)

			self.dist_from_obj = self.laser_data[270]
			self.drive_object()

			self.turn_avoid(2, -.3)

		#turns around if completely surrounded in objects
		else:
			print("180")
			self.turn_avoid(6, .3)

	#drives long obstacle to avoid until cannot see it anymore
	def drive_object(self):
		while self.laser_data <= self.dist_from_obj+.2:
				self.pub.publish(Twist(Vector3(.2,0,0),Vector3(0,0,0)))

	#turns to go around object, turns for time length t with power and direction dir
	def turn_avoid(self, t, dir):
		start = time.time()
		while time.time()-start<t: 
			self.pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,dir)))
			time.sleep(1)



	def run(self):
		#making sure lidar has data before running the code
		while self.last_header is None:
			rospy.sleep(0.1)

		#while rospy is running, run this code
		#drive straight until see object, then turn around it
		while not rospy.is_shutdown():
			self.pub.publish(Twist(Vector3(.2,0,0),Vector3(0,0,0)))

			if self.laser_data[0] <= and self.laser_data[0] != 0:
				self.avoid()

			self.update_rate.sleep()


#creates Neato object and runs the run function
if __name__ == "__main__":
	neato1 = Neato()
	neato1.run()