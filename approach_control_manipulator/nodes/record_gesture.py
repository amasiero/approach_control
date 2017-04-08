#!/usr/bin/env python

import rospy
import yaml
import os.path
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import TorqueEnable


position_joint_1 = 0.0
position_joint_2 = 0.0
position_joint_3 = 0.0
position_wrist   = 0.0
position_base    = 0.0

def callback_joint_1(data):
    position_joint_1 = data.current_pos

def callback_joint_2(data):
    position_joint_2 = data.current_pos        

def callback_joint_3(data):
    position_joint_3 = data.current_pos        

def callback_wrist(data):
    position_wrist = data.current_pos        

def callback_base(data):
    position_base = data.current_pos        

def main(args): 
     
     # Subscribers
    rospy.Subscriber('/tilt2_controller/state', JointState, callback_joint_1, queue_size = 1)
    rospy.Subscriber('/tilt3_controller/state', JointState, callback_joint_2, queue_size = 1)
    rospy.Subscriber('/tilt4_controller/state', JointState, callback_joint_3, queue_size = 1)
    rospy.Subscriber('/tilt5_controller/state', JointState, callback_wrist,   queue_size = 1)
    rospy.Subscriber('/tilt_controller/state',  JointState, callback_base,    queue_size = 1)

if __name__ == '__main__':
    rospy.init_node('record_geture', anonymous = True)
    main(sys.argv)
    try:
        rospy.spin()
    except KeboardInterrupt:
        rospy.loginfo('Shutting down node record_gesture')