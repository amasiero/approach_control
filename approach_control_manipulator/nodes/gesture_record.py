#!/usr/bin/env python

import rospy
import yaml
import os.path
import numpy as np
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import TorqueEnable


position_joint_1 = 0
position_joint_2 = 0
position_joint_3 = 0
position_wrist = 0
position_base = 0
moving = [0,0,0,0,0]

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
    # Subscribers
    rospy.Subscriber('/tilt2_controller/state', JointState, callback_joint_1, queue_size = 1)
    rospy.Subscriber('/tilt3_controller/state', JointState, callback_joint_2, queue_size = 1)
    rospy.Subscriber('/tilt4_controller/state', JointState, callback_joint_3, queue_size = 1)
    rospy.Subscriber('/tilt5_controller/state', JointState, callback_wrist,   queue_size = 1)
    rospy.Subscriber('/tilt_controller/state',  JointState, callback_base,    queue_size = 1)


    # Reading yaml file for gesture
    fname = os.path.expanduser('~') + '/catkin_ws/src/approach_control/approach_control_config/config/gestures.yaml'
    stream = open(fname, 'r')
    data = yaml.load(stream)
    keys = data.keys()

    # Calling a name for gesture
    gesture_name = raw_input('Please inform a name for the gesture: ')
    if not gesture_name:
        rospy.logerr('It\'s required name for gesture ... ')
    else:
        if not [x==gesture_name for x in keys]:
            keys.append(gesture_name)
        # Calling user action
        rospy.loginfo('To make a gesture you need to enable torque of the servos.')
        enable_torque = raw_input('Would you like to enable torque? Type: yes | no | press enter to keep the actual torque state ')
        if enable_torque.lower() == 'yes':
            srv_joint_1(True)
            srv_joint_2(True)
            srv_joint_3(True)
            srv_wrist(True)
            srv_base(True)
        elif enable_torque.lower() == 'no':
            srv_joint_1(False)
            srv_joint_2(False)
            srv_joint_3(False)
            srv_wrist(False)
            srv_base(False)
        rospy.loginfo('Move your servos to desired position SLOWLY and go back to rest position to save gesture.')
        data[gesture_name]=[]
        rospy.sleep(0.5)
        while (np.round(np.absolute(max([x for x in moving])),2) == 0.0):
            print [x for x in moving]
            finish_gesture = ''
        rospy.sleep(0.1)

        finish_gesture = ''
        while (finish_gesture.lower() != 'yes' and max([x for x in moving]) > 0):
            rospy.loginfo('Recording ...')
            rospy.sleep(0.1) #record a point every step of 600 ms
            if gesture_name in keys:
                data[gesture_name].append([position_joint_1, position_joint_2, position_joint_3, position_wrist, position_base])
                with open(fname, 'w') as yaml_file:
                    yaml_file.write(yaml.dump(data, default_flow_style = False))
            else:
                data[gesture_name] = [position_joint_1, position_joint_2, position_joint_3, position_wrist, position_base]
                with open(fname, 'w') as yaml_file:
                    yaml_file.write(yaml.dump(data, default_flow_style = False))
        rospy.loginfo('GESTURE SAVED!!!')
if __name__ == '__main__':
    rospy.init_node('record_geture', anonymous = True)
    main()
    try:
        rospy.spin()
    except KeboardInterrupt:
        rospy.loginfo('Shutting down node record_gesture')