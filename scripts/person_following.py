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



class Neato(object):
	def __init__(self):
		rospy.init_node('person_following')
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
		self.viz_pub = rospy.Publisher('vis_marker', Marker, queue_size=5)
		self.sub_laser = rospy.Subscriber('/stable_scan', LaserScan, self.laser_callback)
		rospy.on_shutdown(self.shutdown_func)
		self.update_rate = rospy.Rate(10)
		self.timout = time.time()
		self.laser_data = []
		self.last_header = None	

		self.range = [135, 223]
		self.sum_x = 0
		self.sum_y = 0

		self.dist_from_person = 1
		self.ang = 180


		self.vel = Marker()
		self.vel.ns = "person"
		self.vel.type = self.vel.SPHERE
		self.vel.id = 0

		self.vel.scale.x = .1
		self.vel.scale.y = .1
		self.vel.scale.z = .1
		
		self.vel.color.a = 1.0
		self.vel.color.r = 1.0
		self.vel.color.g = 0.5 
		self.vel.color.b = 0.5


	def laser_callback(self, msg):
		self.laser_data = msg.ranges
		self.last_header = msg.header

	def shutdown_func(self):
		self.pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))


	def find_person(self):
		self.pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,.2)))
		for i in range(self.range[0]+1, self.range[1]):
			print("in range")
			if self.laser_data[i] <= 1 and self.laser_data[i] != 0 :
				if abs(self.laser_data[i-1] - self.laser_data[i]) <= .2 :
					if self.laser_data[i-1] <= self.laser_data[i] and self.laser_data[i-1] <= self.laser_data[i]:
						self.dist_from_person = self.laser_data[i]
						self.ang = i
						print("dist from person")
						print(self.dist_from_person)
						self.turn_to_person()

	def turn_to_person(self):
		while (self.laser_data[180] < self.dist_from_person - 0.1 and self.laser_data[180] > self.laser_data[self.range[0]]) or (self.laser_data[180] > self.dist_from_person + 0.1 and self.laser_data[180] < self.laser_data[self.range[1]]):
			if self.ang > 180 and self.ang != 0:
				print("turn greater than")
				self.pub.publish(Twist(Vector3(-.05,0,0),Vector3(0,0,.3)))

			elif self.ang < 180 and self.ang != 0:
				print("turn less than")
				self.pub.publish(Twist(Vector3(-.05,0,0),Vector3(0,0,-.3)))

		while self.laser_data[180] >= self.dist_from_person - 0.1 and self.laser_data[180] <= self.dist_from_person + 0.1:
			#print("drive")
			self.pub.publish(Twist(Vector3(-.1,0,0),Vector3(0,0,0)))


	def run(self):
		time.sleep(1)
		while not rospy.is_shutdown():
			self.find_person()

			self.vel.header = self.last_header


			self.vel.pose.position.x = self.laser_data[self.ang]*math.cos(math.radians(self.ang)) 
			self.vel.pose.position.y = -(self.laser_data[self.ang]*math.sin(math.radians(self.ang)))
			self.vel.pose.position.z = 0

			self.viz_pub.publish(self.vel)

			self.update_rate.sleep()



if __name__ == "__main__":
	neato1 = Neato()
	neato1.run()






	"""
PLAN

1) Detect person
	a) look in certina area, 135-225 degrees, up to 4 away
	b) look deeper into any two that are close to each other, save the closer of the two
	c) continue through range, save any point from a set that is closer
2) Drive toward that point
	a) rotate towards that point until your 0 has a very similar distance away
	b) movetowards that person
	c) continue same process of rotating to keep the 0 the aproximate distance away from the person







	"""