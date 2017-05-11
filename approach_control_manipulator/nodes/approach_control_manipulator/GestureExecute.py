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

class GestureExecute(smach.State):
    def __init__(self, gesture_name):
        smach.State.__init__(self, outcomes=['success','fail'])
        
        self.gesture_name = gesture_name
        self.vel_max = 0.01
        self.step = 0.15 #execute a point every step time [ms]

        
        #Publishers
        self.joint1 = rospy.Publisher('/tilt2_controller/command', Float64, queue_size = 2) #Junta 1
        self.joint3 = rospy.Publisher('/tilt4_controller/command', Float64, queue_size = 2) #Junta 3
        self.joint2 = rospy.Publisher('/tilt3_controller/command', Float64, queue_size = 2) #Junta 2
        self.joint4 = rospy.Publisher('/tilt5_controller/command', Float64, queue_size = 2) #Pulso
        self.base = rospy.Publisher('/tilt_controller/command', Float64, queue_size = 2) #base

        self.pub_gesture = rospy.Publisher('/gesture/name', String, queue_size = 10)

        # Services
        rospy.wait_for_service('/tilt2_controller/torque_enable')
        rospy.wait_for_service('/tilt3_controller/torque_enable')
        rospy.wait_for_service('/tilt4_controller/torque_enable')
        rospy.wait_for_service('/tilt5_controller/torque_enable')
        rospy.wait_for_service('/tilt_controller/torque_enable')
        rospy.wait_for_service('/tilt2_controller/set_speed')
        rospy.wait_for_service('/tilt3_controller/set_speed')
        rospy.wait_for_service('/tilt4_controller/set_speed')
        rospy.wait_for_service('/tilt5_controller/set_speed')
        rospy.wait_for_service('/tilt_controller/set_speed')

        self.srv_joint_1 = rospy.ServiceProxy('/tilt2_controller/torque_enable', TorqueEnable, persistent = True)
        self.srv_joint_2 = rospy.ServiceProxy('/tilt3_controller/torque_enable', TorqueEnable, persistent = True)
        self.srv_joint_3 = rospy.ServiceProxy('/tilt4_controller/torque_enable', TorqueEnable, persistent = True)
        self.srv_wrist   = rospy.ServiceProxy('/tilt5_controller/torque_enable', TorqueEnable, persistent = True)
        self.srv_base    = rospy.ServiceProxy('/tilt_controller/torque_enable',  TorqueEnable, persistent = True)

        self.srv_speed1 = rospy.ServiceProxy('/tilt2_controller/set_speed', SetSpeed, persistent = True)
        self.srv_speed2 = rospy.ServiceProxy('/tilt3_controller/set_speed', SetSpeed, persistent = True)
        self.srv_speed3 = rospy.ServiceProxy('/tilt4_controller/set_speed', SetSpeed, persistent = True)
        self.srv_speedwrist   = rospy.ServiceProxy('/tilt5_controller/set_speed', SetSpeed, persistent = True)
        self.srv_speedbase    = rospy.ServiceProxy('/tilt_controller/set_speed',  SetSpeed, persistent = True)

    def execute(self, userdata):

        # Reading yaml file for gesture
        fname = os.path.expanduser('~') + '/catkin_ws/src/approach_control/approach_control_config/config/gestures.yaml'
        stream = open(fname, 'r')
        data = yaml.load(stream)
        keys = data.keys()
        rospy.logwarn(keys)
        rospy.loginfo(self.gesture_name)
        
        if not self.gesture_name:
            rospy.logerr('It\'s required name for gesture ... ')
            return 'fail'
        else:
            print 'print else'
            rospy.logwarn('else')
            try:
                print 'print try'
                rospy.logwarn('try')
                self.srv_joint_1(True)
                self.srv_joint_2(True)
                self.srv_joint_3(True)
                self.srv_wrist(True)
                self.srv_base(True)
                self.srv_speed1(self.vel_max)
                self.srv_speed2(self.vel_max)
                self.srv_speed3(self.vel_max)
                self.srv_speedwrist(self.vel_max)
                self.srv_speedbase(self.vel_max)
                rospy.logwarn(self.gesture_name)
                print 'print' + str(self.gesture_name)
                for x in data[self.gesture_name]:
                    rospy.logwarn('Playing ... %s', str(x))
                    print 'Playing ...'
                    rospy.sleep(self.step) #execute a point every step time (ms)
                    self.joint1.publish(x[0])
                    self.joint2.publish(x[1])
                    self.joint3.publish(x[2])
                    self.joint4.publish(x[3])
                    self.base.publish(x[4])
                rospy.logwarn('FINISHED!!!')
                return 'success'
            except Exception as e:
                rospy.logerr(e)
                return 'fail'