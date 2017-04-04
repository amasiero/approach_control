#!/usr/bin/env python

import smach
import rospy
import numpy as np
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState

class CloseGripper(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['success', 'in_progress', 'fail'])
		
		self.joint4 = rospy.Publisher('/tilt6_controller/command', Float64, queue_size = 10)
		self.count = 0
		self.error_default = 0.04
		
		rate = rospy.Rate(5)

	def callback(self, data):
		# Callback da temperatura do servo
		self.pos = data.current_pos
		self.error = data.error

	def execute(self, userdata):
		rospy.loginfo('Closing Gripper\n')
		rospy.sleep(0.1)
		
		rospy.Subscriber('/tilt6_controller/state', JointState, self.callback)
		self.joint4.publish(-0.79)
		
		rospy.sleep(4)
		
		rospy.loginfo('Position: %f', np.round(self.pos, 2))
		rospy.loginfo('Error: %s', np.absolute(self.error))

		if np.absolute(self.error) < self.error_default:
			rospy.loginfo('Gripper closed')
			return 'sucess'
		elif self.count < 1:
			self.count += 1
		else:
			return 'fail'
