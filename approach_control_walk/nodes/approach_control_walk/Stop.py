#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from p2os_msgs.msg import MotorState
from geometry_msgs.msg import Twist

class Stop(smach.State):
	def __init__(self, outcomes=['success','fail'], input_keys=['robot_speed']):
		smach.State.__init__(self, outcomes=['success','fail'], input_keys=['robot_speed'])
		self.motor_state_pub = rospy.Publisher('/cmd_motor_state', MotorState, queue_size=10)
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


	def execute(self, userdata):
		rospy.loginfo('Stopping Robot')
		time.sleep(0.1)

		velocity = Twist()
		motor_state = MotorState()
		motor_state.state = 4
		self.motor_state_pub.publish(motor_state)

		velocity.linear.x = 0.0
		velocity.angular.z = 0.0

		if userdata.robot_speed == 'stop':
			velocity.linear.x = 0.0
			self.cmd_vel_pub.publish(velocity)
			return 'success'
		else:
			rospy.loginfo('Stopping Robot failling -> error: %s' % userdata.robot_speed)
			return 'fail'
