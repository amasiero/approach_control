#!/usr/bin/env python

import rospy
import smach
import numpy as np
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState


class OpenGripper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['success', 'in_progress', 'fail'])

        self.joint4 = rospy.Publisher('/tilt6_controller/command', Float64, queue_size = 10)

        self.count = 0
        self.error_default = 0.04
        self.pos = 0
        self.error = 0

        rospy.Rate(5)

    def  callback(self, data):
        self.pos = data.current_pos
        self.error = data.error

    def execute(self, userdata):
        rospy.loginfo('Opening Gripper')
        rospy.sleep(0.1)

        rospy.Subscriber('/tilt6_controller/state', JointState, self.callback)
        self.joint4.publish(2.61)

        rospy.sleep(4)
        rospy.loginfo('Position: %f', np.round(self.pos, 2))
        rospy.loginfo('Error: %f', np.absolute(self.error))

        if np.absolute(self.error) < self.error_default:
            rospy.loginfo('Gripper open')
            return 'success'
        elif self.count < 1:
            self.count += 1
            return 'in_progress'
        else:
            return 'fail'