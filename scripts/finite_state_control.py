#!/usr/bin/env python

from __future__ import print_function, division
from geometry_msgs.msg import Twist, Vector3
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from neato_node.msg import Bump
import select
import termios
import rospy
import time
import math
import sys
import tty





class Neato(object):

	def __init__(self):
		#initializes rospy the publishers and subscribers, sets up the on shitdown, and initializes a lot of variabels
		rospy.init_node('wall_follow')
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
		self.sub_laser = rospy.Subscriber('/stable_scan', LaserScan, self.laser_callback)
		rospy.on_shutdown(self.shutdown_func)
		self.update_rate = rospy.Rate(10)
		self.timout = time.time()
		self.laser_data = []
		self.last_header = None

		self.dist_right_wall = 1


	def laser_callback(self, msg):
		self.laser_data = msg.ranges


	def shutdown_func(self):
		self.pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))



	#drive straight, continuously check at 330, 300, 270, 240, and 210
	#correct as drive to keep each at correct distance
	def parallel_to_wall(self):
		#if turned away from wall, turn back
		if self.laser_data[120] <= self.laser_data[60]:
			print([self.laser_data[94], self.laser_data[120], self.laser_data[60]])
			self.pub.publish(Twist(Vector3(-.2,0,0),Vector3(0,0,.05)))
			print("forward left turn")
		#if turned towards wall, turn out
		elif self.laser_data[120] >= self.laser_data[60]: 
			print([self.laser_data[94], self.laser_data[120], self.laser_data[60]])
			self.pub.publish(Twist(Vector3(-.2,0,0),Vector3(0,0,-.05)))
			print("forward right turn")
		#if straight, drive straight
		else:
			self.pub.publish(Twist(Vector3(.2,0,0),Vector3(0,0,0)))
			print("drive)")	



	#saw something straight ahead, avoid it
	def avoid(self):
		#turns left around an object if that space appears to be open
		#if there is something to the right and not to the left, it will turn towards the left
		if self.laser_data[94] <= 1 and (self.laser_data[270] >= 1 or self.laser_data[270]  == 0):
			self.turn_avoid(3.1, .3)

			self.dist_from_obj = self.laser_data[94]
			self.drive_object()

			self.turn_avoid(3, -.3)

		#turns right around an object if that space appears to be open
		#if there is something to the right and not to the left, it will turn towards the left		
		elif (self.laser_data[94] >= 1 or self.laser_data[270]  == 0) and self.laser_data[270] <= 1:
			self.turn_avoid(3.1, -.3)

			self.dist_from_obj = self.laser_data[270]
			self.drive_object()

			self.turn_avoid(3, -.3)

		#turns around if completely surrounded in objects
		else:
			print("180")
			self.turn_avoid(6, .3)

	#drives long obstacle to avoid until cannot see it anymore
	def drive_object(self):
		while self.laser_data[94] <= self.dist_from_obj+.2:
			self.pub.publish(Twist(Vector3(-.2,0,0),Vector3(0,0,0)))

	#turns to go around object, turns for time length t with power and direction dir
	def turn_avoid(self, t, dir):
		start = time.time()
		while time.time()-start<t: 
			self.pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,dir)))
			time.sleep(1)




		
	def run(self):
		while self.laser_data == []:
			rospy.sleep(0.1)
		
		while not rospy.is_shutdown():

			#drive backwards along right wall
			#if object in the way, avoid, follow wall again, distance doesn't matter
			self.parallel_to_wall()
			if self.laser_data[180] <=.4 and self.laser_data[180] != 0:
				self.avoid()

			self.update_rate.sleep()



if __name__ == "__main__":
	neato1 = Neato()
	neato1.run()