#!/usr/bin/env python

import smach
import rospy
import math
import numpy as np
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import SetSpeed
from approach_control_manipulator.utils.arm_mod import Arm3Link

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

        # Create an Arm3Link
        arm = Arm3Link(q0 = self.angle, L = np.array([130,133,225]))
        arm.q = arm.inv_kin(xyz = [userdata.coordX, userdata.coordY, userdata.coordZ])

        if not math.isnan(arm.q[0]):

            # Transformations to interactions with dinamyxels servos
            q1 = np.round(arm.q[0] - (1.92), 2)
            q2 = np.round(arm.q[1] + (0.41), 2)
            q3 = np.round(arm.q[2] + (0.46), 2)
            q4 = np.round(arm.q[3] - (0.71), 2)
            self.q_list = [q1, q2, q3]

            # Vector with joint difference angles
            self.diff_vel[0] = abs(abs(self.current_pose_joint1) - abs(q1))
            self.diff_vel[1] = abs(abs(self.current_pose_joint2) - abs(q2))
            self.diff_vel[2] = abs(abs(self.current_pose_joint3) - abs(q3))

            # Sorting differences list
            for x in range(len(self.diff_vel) - 1, 0, -1):
                for i in range(x):
                    if self.diff_vel[i] > self.diff_vel[i + 1]:
                        temp = self.diff_vel[i]
                        temp_aux = self.controllers[i]
                        temp_aux2 = self.q_list[i]
                        self.diff_vel[i] = self.diff_vel[i + 1]
                        self.controllers[i] = self.controllers[i + 1]
                        self.q_list[i] = self.q_list[i + 1]
                        self.diff_vel[i + 1] = temp
                        self.controllers[i + 1] = temp_aux
                        self.q_list[i + 1] = temp_aux2

            # Making the proportion values of speeds
            self.speeds = np.round([(((self.diff_vel[0] * 100) / self.diff_vel[2]) / 100) \
                * self.velocity_lim, (((self.diff_vel[1] * 100) / self.diff_vel[2]) / 100) \
                * self.velocity_lim, self.velocity_lim], 3)

            # Calling services and publishing joint values
            for i in range(len(self.speeds)):
                if self.speeds[i] == 0.0:
                    self.speeds[i] = 0.1
                self.servo_speed[self.controllers[i]](self.speeds[i])
                rospy.loginfo("\nSPEEDS: %s  Joint: %s  Diff: %s \n", str(self.speeds[i]), \
                    str(self.controller[i]), str(self.diff_vel[i]))

            # Publishing joint values
            rospy.loginfo("\n\nQ LIST: %s %s %s %s \n\n", str(q1), str(q2), str(q3), str(q4))
            self.joint1.publish(q1)
            self.joint2.publish(q2)
            self.joint3.publish(q3)
            self.base.publish(q4)
            rospy.sleep(3)

            while (self.moving_joint2 and self.moving_joint3 and self.moving_joint4):
                rospy.loginfo('Moving to point')

            # Errors
            rospy.loginfo('\n Error joint1: %f \n Error joint2: %f \n Error joint3: %f', np.absolute(self.error_joint1), \
                np.absolute(self.error_joint2), np.absolute(self.error_joint3))

        if np.absolute(self.error_joint1) < self.error_default and np.absolute(self.error_joint2) < self.error_default \
                and np.absolute(self.error_joint3) < self.error_default:
            rospy.loginfo('Goal reached')
            self.count = 0
            return 'success'
        elif self.count < 1:
            rospy.loginfo('Sending Goal Again')
            self.count += 1
            rospy.sleep(0.1)
            return 'in_progress'
        elif (math.isnan(arm.q[0])):
            rospy.logerr("\n\n\nNAN VALUE ERROR !!!\n\n\n")
            return 'fail'
        else:
            return 'fail'