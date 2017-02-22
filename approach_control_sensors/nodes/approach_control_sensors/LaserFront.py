#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from sensor_msgs.msg import LaserScan

class LaserFront(smach.State):
	def __init__(self, outcomes=['far', 'closer']):
		smach.State.__init__(self, outcomes=['far', 'closer'])
		rospy.Subscriber('/scan', LaserScan, readLaser)
		self.distance = 0.0

	def readLaser(self, data):
		self.distance = min(data.ranges[57:114])

	def execute(self, userdata):
		rospy.loginfo('Read front range laser...')
		if self.distance > 0.5:
			return 'far'
		else
			return 'closer'
