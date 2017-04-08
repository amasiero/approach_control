#!/usr/bin/env python

import rospy
import yaml
import os.path
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import TorqueEnable


position_joint_1
position_joint_2
position_joint_3
position_wrist  
position_base   

def callback_joint_1(data):
    global position_joint_1
    position_joint_1 = data.current_pos

def callback_joint_2(data):
    global position_joint_2
    position_joint_2 = data.current_pos        

def callback_joint_3(data):
    global position_joint_3       
    position_joint_3 = data.current_pos        

def callback_wrist(data):
    global position_wrist
    position_wrist = data.current_pos        

def callback_base(data):
    global position_base
    position_base = data.current_pos        

def main(args): 
     global position_joint_1, position_joint_2, position_joint_3, position_wrist, position_base

     # Subscribers
    rospy.Subscriber('/tilt2_controller/state', JointState, callback_joint_1, queue_size = 1)
    rospy.Subscriber('/tilt3_controller/state', JointState, callback_joint_2, queue_size = 1)
    rospy.Subscriber('/tilt4_controller/state', JointState, callback_joint_3, queue_size = 1)
    rospy.Subscriber('/tilt5_controller/state', JointState, callback_wrist,   queue_size = 1)
    rospy.Subscriber('/tilt_controller/state',  JointState, callback_base,    queue_size = 1)

    # Services
    srv_joint_1 = rospy.ServiceProxy('/tilt2_controller/torque_enable', TorqueEnable, persistent=True)
    srv_joint_2 = rospy.ServiceProxy('/tilt3_controller/torque_enable', TorqueEnable, persistent=True)
    srv_joint_3 = rospy.ServiceProxy('/tilt4_controller/torque_enable', TorqueEnable, persistent=True)
    srv_wrist = rospy.ServiceProxy('/tilt5_controller/torque_enable', TorqueEnable, persistent=True)
    srv_base = rospy.ServiceProxy('/tilt_controller/torque_enable',  TorqueEnable, persistent=True)

if __name__ == '__main__':
    rospy.init_node('record_geture', anonymous = True)
    main(sys.argv)
    try:
        rospy.spin()
    except KeboardInterrupt:
        rospy.loginfo('Shutting down node record_gesture')