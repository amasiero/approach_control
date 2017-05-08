#!/usr/bin/env python

import rospy
import yaml
import os.path
import numpy as np
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import TorqueEnable, SetSpeed

position_joint_1 = 0
position_joint_2 = 0
position_joint_3 = 0
position_wrist = 0
position_base = 0
moving = [0,0,0,0,0]
vel_max = 0.6

def callback_joint_1(data):
    global position_joint_1,moving
    position_joint_1 = data.current_pos
    moving[0] = data.velocity

def callback_joint_2(data):
    global position_joint_2,moving
    position_joint_2 = data.current_pos
    moving[1] = data.velocity


def callback_joint_3(data):
    global position_joint_3,moving
    position_joint_3 = data.current_pos
    moving[2] = data.velocity

def callback_wrist(data):
    global position_wrist,moving
    position_wrist = data.current_pos
    moving[3] = data.velocity

def callback_base(data):
    global position_base,moving
    position_base = data.current_pos
    moving[4] = data.velocity

def main():
    # Giving global scope to servo position variables
    global position_joint_1, position_joint_2, position_joint_3, position_wrist, position_base

    # Services
    srv_joint_1 = rospy.ServiceProxy('/tilt2_controller/torque_enable', TorqueEnable, persistent=True)
    srv_joint_2 = rospy.ServiceProxy('/tilt3_controller/torque_enable', TorqueEnable, persistent=True)
    srv_joint_3 = rospy.ServiceProxy('/tilt4_controller/torque_enable', TorqueEnable, persistent=True)
    srv_wrist   = rospy.ServiceProxy('/tilt5_controller/torque_enable', TorqueEnable, persistent=True)
    srv_base    = rospy.ServiceProxy('/tilt_controller/torque_enable',  TorqueEnable, persistent=True)
    srv_speed1 = rospy.ServiceProxy('/tilt2_controller/set_speed', SetSpeed, persistent=True)
    srv_speed2 = rospy.ServiceProxy('/tilt3_controller/set_speed', SetSpeed, persistent=True)
    srv_speed3 = rospy.ServiceProxy('/tilt4_controller/set_speed', SetSpeed, persistent=True)
    srv_speedwrist   = rospy.ServiceProxy('/tilt5_controller/set_speed', SetSpeed, persistent=True)
    srv_speedbase    = rospy.ServiceProxy('/tilt_controller/set_speed',  SetSpeed, persistent=True)
    # Subscribers
    rospy.Subscriber('/tilt2_controller/state', JointState, callback_joint_1, queue_size = 1)
    rospy.Subscriber('/tilt3_controller/state', JointState, callback_joint_2, queue_size = 1)
    rospy.Subscriber('/tilt4_controller/state', JointState, callback_joint_3, queue_size = 1)
    rospy.Subscriber('/tilt5_controller/state', JointState, callback_wrist,   queue_size = 1)
    rospy.Subscriber('/tilt_controller/state',  JointState, callback_base,    queue_size = 1)
    #Publishers
    joint1 = rospy.Publisher('/tilt2_controller/command', Float64, queue_size=1) #Junta 1
    joint3 = rospy.Publisher('/tilt4_controller/command', Float64, queue_size=1) #Junta 3
    joint2 = rospy.Publisher('/tilt3_controller/command', Float64, queue_size=1) #Junta 2
    joint4 = rospy.Publisher('/tilt5_controller/command', Float64, queue_size = 1) #Pulso
    base = rospy.Publisher('/tilt_controller/command', Float64, queue_size = 1)   #base

    # Reading yaml file for gesture
    fname = os.path.expanduser('~') + '/catkin_ws/src/approach_control/approach_control_config/config/gestures.yaml'
    stream = open(fname, 'r')
    data = yaml.load(stream)
    keys = data.keys()

    # Calling a name for gesture
    gesture_name = "point"
    if not gesture_name:
        rospy.logerr('It\'s required name for gesture ... ')
    else:
            srv_joint_1(True)
            srv_joint_2(True)
            srv_joint_3(True)
            srv_wrist(True)
            srv_base(True)
            srv_speed1(vel_max)
            srv_speed2(vel_max)
            srv_speed3(vel_max)
            srv_speedwrist(vel_max)
            srv_speedbase(vel_max)
            for x in data[gesture_name]:
                rospy.loginfo('Playing ...')
                rospy.sleep(0.2) #record a point every step of 600 ms
                joint1.publish(x[0])
                joint2.publish(x[1])
                joint3.publish(x[2])
                joint4.publish(x[3])
                base.publish(x[4])
        # rospy.loginfo('FINISHED!!!')
if __name__ == '__main__':
    rospy.init_node('record_geture', anonymous = True)
    main()
    try:
        rospy.spin()
    except KeboardInterrupt:
        rospy.loginfo('Shutting down node record_gesture')