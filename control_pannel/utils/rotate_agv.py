#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time

def rotate_agv(interval):
	rospy.init_node('rotate_agv')

	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

	# Rotate message
	rot = Twist()
	rot.angular.z = 0.5

	# set the publish rate in hertz
	rotate_rate = rospy.Rate(5)

	start = time.time()
	while time.time() - start < interval:
		pub.publish(rot)
		rotate_rate.sleep()
	