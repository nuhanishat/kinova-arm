#! /usr/bin/env python

# Author: Nuha Nishat
# Date: 12/30/20

import rospy
import sys
import math

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

import geometry_msgs.msg
from math import pi 
from std_msgs.msg import String

import kinova_msgs.msg
from kinova_msgs.msg import JointVelocity, PoseVelocity



class VelcityControl():
	def __init__(self):
		rospy.init_node('drivercontrol')
		rate = rospy.Rate(100)

		self.pub = rospy.Publisher('/j2s7s300_driver/in/cartesian_velocity', PoseVelocity, queue_size=10)


	def commander(self, distance, speed):
		vel = PoseVelocity()

		beginTime = rospy.Time.now()
		endTime = rospy.Duration(distance/float(abs(speed))) + beginTime

		while rospy.Time.now()< beginTime:
			vel.twist_linear_x = speed
			self.pub.publish(vel)


if __name__ == '__main__':
	v = VelcityControl()

	v.commander(0.4, 0.05)