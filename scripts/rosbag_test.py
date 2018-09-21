#!/usr/bin/env python

from visualization_msgs.msg import Marker
import rospy


class VizBag(object):
	def __init__(self):
		rospy.init_node('rosbag_test')
		self.pub = rospy.Publisher('vis_marker', Marker, queue_size=5)
		self.update_rate = rospy.Rate(0.1)

		#Python marker definition from https://www.programcreek.com/python/example/88812/visualization_msgs.msg.Marker

		self.vel = Marker()
		self.vel.header.frame_id = "odom"
		self.vel.header.stamp = rospy.Time.now()
		self.vel.ns = "testing_markers"
		self.vel.action = self.vel.ADD
		self.vel.type = self.vel.SPHERE
		self.vel.id = 0

		self.vel.scale.x = 1
		self.vel.scale.y = 1
		self.vel.scale.z = 1

		self.vel.pose.position.x = 1
		self.vel.pose.position.y = 2
		self.vel.pose.position.z = 0
		

		self.vel.color.a = 1.0
		self.vel.color.r = 1.0
		self.vel.color.g = 0.5 
		self.vel.color.b = 0.5

	def publish(self):
		while not rospy.is_shutdown():
			self.pub.publish(self.vel)
			self.update_rate.sleep()

if __name__ == "__main__":
	viz = VizBag()
	viz.publish()


