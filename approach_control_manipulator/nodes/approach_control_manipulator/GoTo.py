#!/usr/bin/env python

import smach
import rospy
import math
import numpy as np
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import SetSpeed

class GoTo(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes = ['success', 'in_progress','fail'], input_keys = ['coordX', 'coordY', 'coordZ'])

		self.joint_position = dict()
		self.count = 0
		self.error_default = 0.04 # error for joint angles
		self.diff_vel = [0, 0, 0] # list to receive difference betwween the last and the next angle values
		self.speed = [0, 0, 0,] # list to receive speeds
		self.velocity_limit = 0.8 # max velocity = 2.0
		self.default_joint_speed = 0.3
		self.angle = np.array([math.pi / 2, -math.pi / 2, 0, math.pi /2 ])
		self.angles_now = self.angle
		self.servo_speed = dict()
		self.controllers = ['tilt2_controller', 'tilt3_controller', 'tilt4_controller']

		#Publishers
		self.joint1 = rospy.Publisher('/tilt2_controller/command', Float64, queue_size = 1) # Joint 1
		self.joint2 = rospy.Publisher('/tilt3_controller/command', Float64, queue_size = 1) # Joint 2
		self.joint3 = rospy.Publisher('/tilt4_controller/command', Float64, queue_size = 1) # Joint 3
		self.joint4 = rospy.Publisher('/tilt5_controller/command', Float64, queue_size = 1) # Wrist
		self.base = rospy.Publisher('/tilt_controller/command', Float64, queue_size = 1) # Base

		rospy.Rate(5)

	def callback_joint1(self, data):
		self.pos_joint1 = data.current_pos
		self.error_joint1 = data.error
		self.moving_joint1 = data.is_moving

	def callback_joint2(self, data):
		self.pos_joint2 = data.current_pos
		self.error_joint2 = data.error
		self.moving_joint2 = data.is_moving
	
	def callback_joint3(self, data):
		self.pos_joint3 = data.current_pos
		self.error_joint3 = data.error
		self.moving_joint3 = data.is_moving

	def execute(self, userdata):
		rospy.loginfo('Moving to point')
		rospy.sleep(0.3)

		# Subscribers
		rospy.Subscriber('/tilt2_controller/state', JointState, self.callback_joint1, queue_size = 1)
		rospy.Subscriber('/tilt3_controller/state', JointState, self.callback_joint2, queue_size = 1)
		rospy.Subscriber('/tilt4_controller/state', JointState, self.callback_joint3, queue_size = 1)

		# Making and calling joint speed services with a default speed
		for controller in (self.controllers):
			set_speed_service = '/' + controller + '/set_speed'
			rospy.wait_for_service(set_speed_service)
			self.servo_speed[controller] = rospy.ServiceProxy(set_speed_service, SetSpeed, persistent = True)
			self.servo_speed[controller](self.default_joint_speed)

		# Current position
		self.current_pose_joint1 = self.pos_joint1
		self.current_pose_joint2 = self.pos_joint2
		self.current_pose_joint3 = self.pos_joint3

		self.angles_now[0] = np.round(self.current_pose_joint1 - (1.98), 2)
		self.angles_now[1] = np.round(self.current_pose_joint2 + (0.41), 2)
		self.angles_now[2] = np.round(self.current_pose_joint3 + (0.46), 2)