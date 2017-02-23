#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from sensor_msgs.msg import LaserScan

class LaserFront(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['closerFront', 'closerRight', 'closerLeft', 'far'])
		rospy.Subscriber('/scan', LaserScan, readLaser)
		self.ranges = 0.0

	def readLaser(self, data):
		self.ranges = data.ranges

	def execute(self, userdata):
		rospy.loginfo('Read front range laser...')
		if min(self.ranges[0:57]) < 0.5:
			return 'closerRight'
		elif min(self.ranges[57:114]) < 0.5:
			return 'closerFront'
		elif min(self.ranges[114:171]) < 0.5:
			return 'closerLeft'
		else
			return 'far'
