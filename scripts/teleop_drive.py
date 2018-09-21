#!/usr/bin/env python

from __future__ import print_function, division
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
import termios
import select
import rospy
import sys
import tty




# Got help from Anil Patel, Nina Tchirkova, and Matt Brucker
# Some code taken from Paul's neato code

"""

This code is a teleop code that allows the user to control the robot with "W," "S," A," "D," and "F"

"""

class Neato(object):
	def __init__(self):

		rospy.init_node('teleop_drive')
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
		self.update_rate = rospy.Rate(10)

		self.settings = termios.tcgetattr(sys.stdin)

		self.key = None


	#code from Paul to determine which key is which when it is selected
	def getKey(self):
	    tty.setraw(sys.stdin.fileno())
	    select.select([sys.stdin], [], [], 0)
	    key = sys.stdin.read(1)
	    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
	    return key
	


	def drive(self):
		print("in drive")
		while self.key != '\x03':
			#self.key is the current key pressed
			self.key = self.getKey()

			#forward with "w"
			if self.key == "w":
				print("forward")
				self.pub.publish(Twist(Vector3(1,0,0),Vector3(0,0,0)))
			#backward with "s"
			if self.key == "s": 
				print("backward")
				self.pub.publish(Twist(Vector3(-1,0,0),Vector3(0,0,0)))
			#right with "d"
			if self.key == "d": 
				print("right")
				self.pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,-1)))
			#left with "a"
			if self.key == "a":
				print("left")
				self.pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,1)))
			#stop with "f"
			if self.key == "f":
				print("stop")
				self.pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))

		#if control c is pressed, ros should shutdown
		rospy.signal_shutdown("control c")


	#run runs the teleop drive function while rospy is running
	def run(self):
		while not rospy.is_shutdown():
			self.drive()


if __name__ == "__main__":
	#create a Neato object and run it
	neato1 = Neato()
	neato1.run()
