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


#help creating wall viz from Eric and https://answers.ros.org/question/203782/rviz-marker-line_strip-is-not-displayed/

"""

This code allows the robot to autonomously follow a wall using the lidar scanner

"""


class Neato(object):

	#initializes rospy the publishers and subscribers, sets up the on shitdown, and initializes a lot of variabels
	def __init__(self):
		rospy.init_node('wall_follow')
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
		self.viz_pub = rospy.Publisher('vis_marker', Marker, queue_size=5)
		self.sub_laser = rospy.Subscriber('/stable_scan', LaserScan, self.laser_callback)
		rospy.on_shutdown(self.shutdown_func)
		self.update_rate = rospy.Rate(10)
		self.timout = time.time()
		self.laser_data = []
		self.last_header = None #to be used for the visualization header

		self.dist_from_wall = .5 #if using "line-up" this is the distance the robo should stop from the wall
		self.dist_right_wall = 1 #this will be updated almost immediately, this will be equivalent to the starting distance from the wall when parallel



		self.vel = Marker()
		self.vel.ns = "wall_line"
		self.vel.type = self.vel.LINE_STRIP
		self.vel.id = 0

		self.vel.scale.x = .1
		
		self.vel.color.a = 1.0
		self.vel.color.r = 1.0
		self.vel.color.g = 0.5 
		self.vel.color.b = 0.5

	#saves list of laser data
	#initializes header data for the visualization 
	def laser_callback(self, msg):
		self.laser_data = msg.ranges
		self.last_header = msg.header


	#when rospy is shutdown, stop robot
	def shutdown_func(self):
		self.pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))




	#this function was used to drive the robot to the wall and line it up
	#unfortunately it was very unreliable to I left it out of the final run
	def line_up(self):
		time.sleep(1)
		old_dist = self.laser_data[94] - 10

		#drives robot to wall
		while self.laser_data[180] >= self.dist_from_wall:
			self.pub.publish(Twist(Vector3(-.2,0,0),Vector3(0,0,0)))
			print("drive")

		#turns robot to be parallel with wall
		while self.laser_data[94] - old_dist >= -0 or self.laser_data[94] == 0:
			old_dist = self.laser_data[94]

			self.pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,.5)))
			print("turn")

		self.pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))


	#drive straight, continuously check at 330, 300, 270, 240, and 210
	#correct as drive to keep each at correct distance
	def parallel_to_wall(self):
		#compare length of lidar measurements from -60 degrees (60 degrees) and positive 60 degrees (120 degrees) from perpendicular to wall lidar reading (90 degrees)
		# if the positive angle has a shorter lenght, it should turn away from the wall
		#because of the lidar on the robo I was using, I had to use 94 degrees as theangle perpendicular to the wall because 90-93 id dnot provide lidar readings
		if self.laser_data[120] <= self.laser_data[60]: #self.laser_data[94] > self.dist_right_wall: # or self.laser_data[120] > self.cos_30 or self.laser_data[150] > self.cos_60 or self.laser_data[60] < self.cos_30 or self.laser_data[34] < self.cos_60:
			self.pub.publish(Twist(Vector3(-.2,0,0),Vector3(0,0,.05)))
			print("forward left turn")
		#if the negative angle had a shorter length, it should turn back towards the wall
		elif self.laser_data[120] >= self.laser_data[60]: #self.laser_data[94] < self.dist_from_wall: # or self.laser_data[120] < self.cos_30 or self.laser_data[150] > self.cos_60 or self.laser_data[60] > self.cos_30 or self.laser_data[34] > self.cos_60:
			self.pub.publish(Twist(Vector3(-.2,0,0),Vector3(0,0,-.05)))
			print("forward right turn")
		#if straight, drive straight
		#if there is no difference, simply drive straight
		else:
			self.pub.publish(Twist(Vector3(.2,0,0),Vector3(0,0,0)))
			print("drive)")	


	#determine a beginning and ending point to creat a wall visualization in rviz
	#this will be a constantly updating line that should be over the line found by the lidar
	#my beginning and ending points will be those at 60 and 120 degrees since those will create the line the robot is looking at
	def set_up_marker(self):
		self.vel.points = []
		self.vel.header = self.last_header

		#point position for 60 degree
		first_point = Point()
		first_point.y = -(self.laser_data[60]*math.sin(math.pi/3))
		first_point.x = self.laser_data[60]*math.cos(math.pi/3)
		first_point.z = 0
		self.vel.points.append(first_point)

		#point position for 120 degree
		last_point = Point()
		last_point.y = -(self.laser_data[120]*math.sin(2*math.pi/3))
		last_point.x = self.laser_data[120]*math.cos(2*math.pi/3)
		last_point.z = 0
		self.vel.points.append(last_point)



		
	def run(self):
		while self.last_header is None:
			rospy.sleep(0.1)
		
		while not rospy.is_shutdown():

			#drive backwards along right wall
			#if object in the way, avoid, follow wall again, distance doesn't matter
			self.parallel_to_wall()

			#publish wall visualizer to rviz
			self.set_up_marker()
			self.viz_pub.publish(self.vel)


			self.update_rate.sleep()



if __name__ == "__main__":
	neato1 = Neato()
	neato1.run()




#PRELIMINARY WRITEUP (aka the ideas that should be included)
"""
The goal is to follow the wall, I tried to do this by starting off easy, 
having the robot drive towards the wall until it is 1m away. I then have
the robo turn left in order to get parallel with the wall. I knows if it
is parallel if the 2070 measurement is now 1m. THen the robot must move
parallel to the wall. TO do this, I continuously check the distances at
330, 300, 270, 240, and 210 to make sure they have the propper values 
(within some error range, to be determined). If the values pointed to the
robo facing more towards the wall, the robot will turn slightly left. The
same goes for if the robot is turned too far awar from the wall, it will
then adjust by trying to turn more towards the wall.
"""