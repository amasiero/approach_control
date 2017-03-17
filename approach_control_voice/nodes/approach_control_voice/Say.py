#!/usr/bin/env/ python

import smach_ros
import smach
import rospy
import time
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient

class Say(smach.State):
	def __init__(self, text_to_say = ''):
		smach.State.__init__(self, outcomes=['spoke', 'mute'])
		self.text_to_say = text_to_say
		self.say_pub = rospy.Publisher('/speech', String, queue_size=5)

		rate = rospy.Rate(5)

	def execute(self, userdata):
		time.sleep(0.3)

		if self.text_to_say:
			self.say_pub.publish(self.text_to_say)
			time_now = rospy.get_time()
			while (rospy.get_time() - time_now) <= 4:
				continue
			return 'spoke'
		else:
			return 'mute'