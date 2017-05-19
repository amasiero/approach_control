#!/usr/bin/env python
import rospy
import yaml
import os.path
import numpy as np

from std_msgs.msg import Float64, String
from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import TorqueEnable, SetSpeed
from approach_control_srv.srv import Gesture

def play_gesture( request ):
    try:
        #Publishers
        joint1 = rospy.Publisher('/tilt2_controller/command', Float64, queue_size=1) #Junta 1
        joint3 = rospy.Publisher('/tilt4_controller/command', Float64, queue_size=1) #Junta 3
        joint2 = rospy.Publisher('/tilt3_controller/command', Float64, queue_size=1) #Junta 2
        # joint4 = rospy.Publisher('/tilt5_controller/command', Float64, queue_size = 1) #Pulso
        base = rospy.Publisher('/tilt_controller/command', Float64, queue_size = 1)   #base
        
        # Reading yaml file for gesture
        fname = os.path.expanduser('~') + '/catkin_ws/src/approach_control/approach_control_config/config/gestures.yaml'
        stream = open(fname, 'r')
        f = yaml.load(stream)
        
        for x in f[request.name]:
            rospy.loginfo('Playing ...')
            rospy.sleep(0.2) #record a point every step of 600 ms
            joint1.publish(x[0])
            joint2.publish(x[1])
            joint3.publish(x[2])
            # joint4.publish(x[3])
            base.publish(x[4])
        rospy.loginfo('FINISHED!')
        return True
    except Exception as e:
        rospy.logerr(e)
        return False


rospy.init_node('gesture_server')

# Services
rospy.wait_for_service('/tilt2_controller/torque_enable')
rospy.wait_for_service('/tilt3_controller/torque_enable')
rospy.wait_for_service('/tilt4_controller/torque_enable')
# rospy.wait_for_service('/tilt5_controller/torque_enable')
rospy.wait_for_service('/tilt_controller/torque_enable')
rospy.wait_for_service('/tilt2_controller/set_speed')
rospy.wait_for_service('/tilt3_controller/set_speed')
rospy.wait_for_service('/tilt4_controller/set_speed')
# rospy.wait_for_service('/tilt5_controller/set_speed')
rospy.wait_for_service('/tilt_controller/set_speed')

srv_joint_1 = rospy.ServiceProxy('/tilt2_controller/torque_enable', TorqueEnable, persistent=True)
srv_joint_2 = rospy.ServiceProxy('/tilt3_controller/torque_enable', TorqueEnable, persistent=True)
srv_joint_3 = rospy.ServiceProxy('/tilt4_controller/torque_enable', TorqueEnable, persistent=True)
# srv_wrist   = rospy.ServiceProxy('/tilt5_controller/torque_enable', TorqueEnable, persistent=True)
srv_base    = rospy.ServiceProxy('/tilt_controller/torque_enable',  TorqueEnable, persistent=True)
srv_speed1 = rospy.ServiceProxy('/tilt2_controller/set_speed', SetSpeed, persistent=True)
srv_speed2 = rospy.ServiceProxy('/tilt3_controller/set_speed', SetSpeed, persistent=True)
srv_speed3 = rospy.ServiceProxy('/tilt4_controller/set_speed', SetSpeed, persistent=True)
# srv_speedwrist   = rospy.ServiceProxy('/tilt5_controller/set_speed', SetSpeed, persistent=True)
srv_speedbase    = rospy.ServiceProxy('/tilt_controller/set_speed',  SetSpeed, persistent=True)

vel_max = 0.6

srv_joint_1(True)
srv_joint_2(True)
srv_joint_3(True)
# srv_wrist(True)
srv_base(True)
srv_speed1(vel_max)
srv_speed2(vel_max)
srv_speed3(vel_max)
# srv_speedwrist(vel_max)
srv_speedbase(vel_max)

service = rospy.Service('gesture_execution', Gesture, play_gesture)

rospy.spin()
