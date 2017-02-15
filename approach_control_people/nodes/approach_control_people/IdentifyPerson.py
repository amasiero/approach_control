#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time

class IdentifyPerson(smach.State):
	def __init__(self, outcomes=['found', 'closer', 'far', 'fail']):
		smach.State.__init__(self, outcomes=['found', 'closer', 'far', 'fail'])


	def execute(self, userdata):
		rospy.loginfo('Identifying People')