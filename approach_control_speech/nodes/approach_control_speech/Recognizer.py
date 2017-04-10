#!/usr/bin/env python

import rospy
import smach
from dragonfly_speech_recognition.srv import GetSpeech

class Recognizer(smach.State):
	def __init__(self, spec = '', choices = {'0'}, time_out = 0):
		outcomes = ['fail']
		[outcomes.append(x) for x in spec]
		smach.State.__init__(self, outcomes = outcomes)

		self.spec = '|'.join(spec)
		self.choices = ('id', 'values', '0', '0')
		self.time_out = GetSpeech()
		self.time_out.secs = time_out
		self.time_out.nsecs = 0

		rospy.wait_for_service('speech_client/get_speech')
		self.recognized = rospy.ServiceProxy('speech_client/get_speech', GetSpeech)

		rate = rospy.Rate(5)

	def execute(self, userdata):
		rospy.sleep(0.3)
		try:
			result = self.recognized(spec = self.spec, time_out = self.time_out)
			return result.result
		except rospy.ServiceException as exc:
			rospy.logerr('Service did not process resquest: ' + str(exc))
			return 'fail'