#!/usr/bin/env python

import smach
import rospy
import numpy as np
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState

class Rest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','in_progress','fail'], input_keys=['coordX','coordY'])

        # Publishers
        self.joint1 = rospy.Publisher('/tilt2_controller/command', Float64, queue_size=10) #Junta 1
        self.joint2 = rospy.Publisher('/tilt3_controller/command', Float64, queue_size=10) #Junta 2
        self.joint3 = rospy.Publisher('/tilt4_controller/command', Float64, queue_size=10) #Junta 3
        self.joint4 = rospy.Publisher('/tilt5_controller/command', Float64, queue_size=10) #Pulso
        self.base = rospy.Publisher('/tilt_controller/command', Float64, queue_size=10)   #base

        self.count = 0
        self.error_default = 0.04
        self.angle = np.array([math.pi/2, -math.pi/2, 0])
        self.rest_goal = np.array([0.7567,-2.3418,1.9072])
        
        rate = rospy.Rate(5)

    def callback_joint2(self, data):
        self.pos_joint2 = data.current_pos
        self.error_joint2 = data.error
        self.moving_joint2 = data.is_moving

    def callback_joint3(self, data):
        self.pos_joint3 = data.current_pos
        self.error_joint3 = data.error
        self.moving_joint3 = data.is_moving

    def callback_joint4(self, data):
        self.pos_joint4 = data.current_pos
        self.error_joint4 = data.error
        self.moving_joint4 = data.is_moving


    def execute(self, userdata):
        rospy.sleep(0.3)

        # Subscribers
        rospy.Subscriber('/tilt2_controller/state', JointState, self.callback_joint2)     
        rospy.Subscriber('/tilt3_controller/state', JointState, self.callback_joint3)     
        rospy.Subscriber('/tilt4_controller/state', JointState, self.callback_joint4)     

        #Publishing joint values
        self.joint1.publish(self.rest_goal[0])
        self.joint2.publish(self.rest_goal[1])
        self.joint3.publish(self.rest_goal[2])
        rospy.sleep(1)

        while (self.moving_joint2 and self.moving_joint3 and self.moving_joint4):
            rospy.loginfo('Moving to point')

        # Errors
        rospy.loginfo('\n Error joint2: %f \n Error joint3: %f \n Error joint4: %f', np.absolute(self.error_joint2), \
                np.absolute(self.error_joint3), np.absolute(self.error_joint4))
        
        if np.absolute(self.error_joint2) < self.error_default and np.absolute(self.error_joint3) < self.error_default \
                and np.absolute(self.error_joint4) < self.error_default:
            rospy.loginfo('Goal reached')
            self.count = 0
            return 'success'
        elif self.count < 1:
            rospy.loginfo('Send Rest Position Again')
            self.count += 1
            rospy.sleep(0.1)
            return 'in_progress'
        else:
            return 'fail'