#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient


class Play:

	def __init__(self):
		self.voice = rospy.get_param("~voice", "voice_cmu_us_clb_arctic_clunits")

		self.sound_handle = SoundClient()

		rospy.sleep(1)
		rospy.Subscriber('/speech', String, self.play_speak)


	def play_speak(self, msg):
		self.sound_handle.stopAll()
		self.sound_handle.say(msg.data, self.voice)
		rospy.sleep(0.01)

if __name__ == '__main__':
	rospy.init_node('play', anonymous=True)
	Play()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")