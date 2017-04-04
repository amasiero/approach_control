#!/usr/bin/env python

import rospy
import smach

class Default(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['success', 'fail'], input_keys = ['coordX', 'coordY', 'coordZ'], output_keys = ['coordX', 'coordY', 'coordZ'])

		rate = rospy.Rate(5)

	def execute(self, userdata):
		rospy.sleep(0.3)

		userdata.coordX = 200
		userdata.coordY = 200
		userdata.coordZ = 0

		if True:
			rospy.loginfo('Position changed')
			return 'success'
		else:
			return 'fail'