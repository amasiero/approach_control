#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Float64

class CheckDistance(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['far', 'closer', 'fail'])
		self.distance = None

	def execute(self, userdata):
	 	