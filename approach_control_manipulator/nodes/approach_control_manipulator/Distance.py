#!/usr/bin/env python

import rospy
import smach
import tf
import math
import numpy as np

class Distance(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes = ['success', 'in_progress','fail'], input_keys = ['coordX', 'coordY', 'coordZ'], output_keys = ['coordX', 'coordY', 'coordZ'])

		self.count = 0
		self.listener = tf.TransformListener()
		self.alpha = 30 * (math.pi / 180) # angle of kinect at robot

		rate = rospy.Rate(5)

	def execute(self, userdata):
		rospy.sleep(0.3)

		try:

			(self.trans, self.rot) = self.listener.lookupTransform('/camera_link', '/object_10', rospy.Time(0))
			self.check = True

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			self.check = False

		if self.check:
			# normalize distances from kinect to object
			userdata.coordX = np.round((self.trans[0] * 1000) * np.cos(self.alpha), 2)
			userdata.coordY = -(np.round((self.trans[2] * 1000) * np.cos(self.alpha), 2))
			userdata.coordZ = np.round((self.trans[1] * 1000), 2)

			rospy.sleep(2)

			self.count = 0
			return 'success'
		elif self.count < 30:
			rospy.loginfo('Next try')
			self.count += 1
			rospy.sleep(0.5)
			return 'in_progress'
		else:
			return 'fail'