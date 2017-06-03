#!/usr/bin/env python

import rospy
import smach
import numpy as np
from std_msgs.msg import String, Float64
from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import SetSpeed

class Tilt(smach.State):
    def __init__(self, tilt=''):
        smach.State.__init__(self, outcomes=['success','fail'])

        self.tilt = tilt
        self.max_vel = 0.6

        self.tilt_head_positions = {'up':1.115,'front':0.0,'down':-0.65, 'more_down':-1.4}


        # Publishers
        self.tilt_head = rospy.Publisher('/tilt_head/command', Float64, queue_size=1) #tilt

        # Subscribers
        rospy.Subscriber('/tilt_head/state', JointState, self.callback, queue_size=1)     #callback para receber dados do servo [id:8]

        #Services
        self.tilt_speed = rospy.ServiceProxy('/tilt_head/set_speed', SetSpeed, persistent=True)
        rate = rospy.Rate(10)

    def callback(self, data):
        self.pos_joint3 = data.current_pos
        self.error_joint3 = data.error
        self.moving_joint3 = data.is_moving

    def execute(self, userdata):
        rospy.sleep(0.3)

        if (self.tilt != ''):
            try:
                self.tilt_speed(self.max_vel)
                self.tilt_head.publish(self.tilt_head_positions[self.tilt])
                rospy.sleep(4)
                return 'success'
            except:
                rospy.loginfo("Speed error")
                return 'fail'
        else:
            rospy.loginfo("Please, complete pan and tilt params ")
            return 'fail'