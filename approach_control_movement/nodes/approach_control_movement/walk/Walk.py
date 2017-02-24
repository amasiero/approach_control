#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from p2os_msgs.msg import MotorState
from geometry_msgs.msg import Twist

class Walk(smach.State):
	def __init__(self, linear=0.0, angular=0.0):
		smach.State.__init__(self, outcomes=['walking', 'stopping'])
		self.motor_state_pub = rospy.Publisher('/cmd_motor_state', MotorState, queue_size=10)
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		self.linear = linear
		self.angular = angular


	def execute(self, userdata=None):
		rospy.loginfo('Robot is walking...')
		time.sleep(0.1)

		velocity = Twist()
		motor_state = MotorState()
		motor_state.state = 4
		self.motor_state_pub.publish(motor_state)

		velocity.linear.x = self.linear
		velocity.angular.z = self.angular
		self.cmd_vel_pub.publish(velocity)
		
		if (self.linear == 0.0 and self.angular == 0.0):
			return 'stopping'

		return 'walking'
		
