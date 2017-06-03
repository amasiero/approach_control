#!/usr/bin/env python

import rospy
import smach
import numpy as np
from std_msgs.msg import String, Float64
from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import SetSpeed

class Pan(smach.State):
    def __init__(self, pan=''):
        smach.State.__init__(self, outcomes=['success','fail'])

        self.pan = pan
        self.max_vel = 0.6

        self.pan_head_positions = {'right':-1.672,'front':-0.215,'left':1.483}
        
        # Publishers
        self.pan_head = rospy.Publisher('/pan_head/command', Float64, queue_size=1) #pan
        
        # Subscribers
        rospy.Subscriber('/pan_head/state', JointState, self.callback, queue_size=1)     #callback para receber dados do servo [id:7]
        
        #Services
        self.pan_speed = rospy.ServiceProxy('/pan_head/set_speed', SetSpeed, persistent=True)
        rate = rospy.Rate(10)

    def callback(self, data):
        self.pos_joint2 = data.current_pos
        self.error_joint2 = data.error
        self.moving_joint2 = data.is_moving

    
    def execute(self, userdata):
        rospy.sleep(0.3)

        if (self.pan != ''):
            try:
                self.pan_speed(self.max_vel)
                self.pan_head.publish(self.pan_head_positions[self.pan])
                rospy.sleep(4)
                return 'success'
            except:
                rospy.loginfo("Speed error")
                return 'fail'
        else:
            rospy.loginfo("Please, complete pan and tilt params ")
            return 'fail'