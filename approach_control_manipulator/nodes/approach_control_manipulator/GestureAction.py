#!/usr/bin/env python

import smach_ros
import smach
from smach import state
import rospy
import yaml
import os.path
import numpy as np
from std_msgs.msg import Float64, String
from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import TorqueEnable, SetSpeed
from dynamixel_driver.dynamixel_io import DynamixelIO
from approach_control_srv.srv import Gesture

class GestureAction(smach.State):
    def __init__(self, gesture_name):
        smach.State.__init__(self, outcomes=['success','fail'])

        self.joint_position = [0,0,0,0,0] #List of positions: [Joint 1, Joint 2, Joint 3, Wrist, Base]
        self.moving = [0,0,0,0,0]  #List of velocity: [Joint 1, Joint 2, Joint 3, Wrist, Base]
        self.gesture_name = gesture_name
        self.vel_max = 0.01
        self.step = 0.15 #execute a point every step time [ms]

        # Services
        # self.srv_joint_1 = rospy.ServiceProxy('/tilt2_controller/torque_enable', TorqueEnable, persistent = True)
        # self.srv_joint_2 = rospy.ServiceProxy('/tilt3_controller/torque_enable', TorqueEnable, persistent = True)
        # self.srv_joint_3 = rospy.ServiceProxy('/tilt4_controller/torque_enable', TorqueEnable, persistent = True)
        # self.srv_wrist   = rospy.ServiceProxy('/tilt5_controller/torque_enable', TorqueEnable, persistent = True)
        # self.srv_base    = rospy.ServiceProxy('/tilt_controller/torque_enable',  TorqueEnable, persistent = True)

        # self.srv_speed1 = rospy.ServiceProxy('/tilt2_controller/set_speed', SetSpeed, persistent = True)
        # self.srv_speed2 = rospy.ServiceProxy('/tilt3_controller/set_speed', SetSpeed, persistent = True)
        # self.srv_speed3 = rospy.ServiceProxy('/tilt4_controller/set_speed', SetSpeed, persistent = True)
        # self.srv_speedwrist   = rospy.ServiceProxy('/tilt5_controller/set_speed', SetSpeed, persistent = True)
        # self.srv_speedbase    = rospy.ServiceProxy('/tilt_controller/set_speed',  SetSpeed, persistent = True)
        # Subscribers
        # rospy.Subscriber('/tilt2_controller/state', JointState, self.callback_joint_1)
        # rospy.Subscriber('/tilt3_controller/state', JointState, self.callback_joint_2)
        # rospy.Subscriber('/tilt4_controller/state', JointState, self.callback_joint_3)
        # rospy.Subscriber('/tilt5_controller/state', JointState, self.callback_wrist)
        # rospy.Subscriber('/tilt_controller/state',  JointState, self.callback_base)
        #Publishers
        # self.joint1 = rospy.Publisher('/tilt2_controller/command', Float64, queue_size = 10) #Junta 1
        # self.joint3 = rospy.Publisher('/tilt4_controller/command', Float64, queue_size = 10) #Junta 3
        # self.joint2 = rospy.Publisher('/tilt3_controller/command', Float64, queue_size = 10) #Junta 2
        # self.joint4 = rospy.Publisher('/tilt5_controller/command', Float64, queue_size = 10) #Pulso
        # self.base = rospy.Publisher('/tilt_controller/command',    Float64, queue_size = 10) #base

        # self.pub_gesture = rospy.Publisher('/gesture/name', String, queue_size = 10)
        rospy.wait_for_service('gesture_execution')
        self.srv_gesture_play = rospy.ServiceProxy('gesture_execution', Gesture)


    def callback_joint_1(self, data):
        self.joint_position[0] = data.current_pos
        self.moving[0] = data.velocity

    def callback_joint_2(self, data):
        self.joint_position[1] = data.current_pos
        self.moving[1] = data.velocity


    def callback_joint_3(self, data):
        self.joint_position[2] = data.current_pos
        self.moving[2] = data.velocity

    def callback_wrist(self, data):
        self.joint_position[3] = data.current_pos
        self.moving[3] = data.velocity

    def callback_base(self, data):
        self.joint_position[4] = data.current_pos
        self.moving[4] = data.velocity


    def execute(self, userdata):
        rospy.sleep(0.3)
        # Reading yaml file for gesture
        fname = os.path.expanduser('~') + '/catkin_ws/src/approach_control/approach_control_config/config/gestures.yaml'
        stream = open(fname, 'r')
        data = yaml.load(stream)
        keys = data.keys()
        rospy.logwarn(keys)

        if not self.gesture_name:
            rospy.logerr('It\'s required name for gesture ... ')
            return 'fail'
        else:
            try:
                # self.srv_joint_1(True)
                # rospy.sleep(0.01)
                # self.srv_joint_2(True)
                # rospy.sleep(0.01)
                # self.srv_joint_3(True)
                # rospy.sleep(0.01)
                # self.srv_wrist(True)
                # rospy.sleep(0.01)
                # self.srv_base(True)
                # motorD = DynamixelIO(port='/dev/ttyUSB1', baudrate='1000000')
                # motorD.set_multi_torque_enabled(( (1, True), (2, True), (3, True), (4, True), (5, True), (6, True)))
                # self.srv_speed1(self.vel_max)
                # # rospy.sleep(0.01)
                # self.srv_speed2(self.vel_max)
                # # rospy.sleep(0.01)
                # self.srv_speed3(self.vel_max)
                # # rospy.sleep(0.01)
                # self.srv_speedwrist(self.vel_max)
                # # rospy.sleep(0.01)
                # self.srv_speedbase(self.vel_max)
                # print(motorD.get_feedback(1))
                # # for x in data[self.gesture_name]:
                #     rospy.loginfo('Playing ...')
                #     rospy.sleep(self.step) #execute a point every step time (ms)
                #     # self.joint1.publish(x[0])
                #     # self.joint2.publish(x[1])
                #     # self.joint3.publish(x[2])
                #     # self.joint4.publish(x[3])
                #     # self.base.publish(x[4])
                #     rospy.loginfo(type(x[0]))
                #     motorD.set_multi_position( ( (1, x[4]), (2, x[0]), (3, x[1]), (4, x[2]), (5, x[3]) ) )
                # rospy.loginfo('FINISHED!!!')

                # self.pub_gesture.publish(self.gesture_name)
                # rospy.sleep(10)
                g = self.srv_gesture_play(self.gesture_name)
                rospy.logwarn(g.executed)
                if g.executed:
                    return 'success'
                else:
                    return 'fail'
            except Exception as e:
                rospy.logerr(e)
                return 'fail'