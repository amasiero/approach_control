#!/usr/bin/env python

import rospy
import rosparam
import smach
import yaml
import os.path
from geometry_msgs.msg import PoseWithCovarianceStamped

class SetInitialPosition(smach.State):

    def __init__(self, local = 'Initial Position', var = 0.05):
        smach.State.__init__(self, outcomes = ['success', 'fail'])

        # Reading yaml file for gesture
        fname = os.path.expanduser('~') + '/catkin_ws/src/approach_control/approach_control_config/config/locals.yaml'
        stream = open(fname, 'r')
        self.data = yaml.load(stream)
        self.local = local
        self.keys = self.data.keys()
        self.initialPositionMsg = PoseWithCovarianceStamped()
        self.initPose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 10)
        self.var = var
                

    def execute(self, userdata):
        
        try:
                    
            if self.local in self.keys:
                local_values =  self.data[self.local] 
                rospy.loginfo(local_values)
                # Define the initial position
                self.initialPositionMsg.pose.pose.position.x = local_values[0][0]
                self.initialPositionMsg.pose.pose.position.y = local_values[0][1]
                self.initialPositionMsg.pose.pose.orientation.w = local_values[1][3]
                self.initialPositionMsg.pose.pose.orientation.z = local_values[1][2]

                self.initialPositionMsg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
                
                # for x in range(0, 35, 5):
                #     self.initialPositionMsg.covariance[x] = self.var
            else:
                rospy.logerr('Local not found!')
            rospy.loginfo('Teste')
            self.initPose.publish(self.initialPositionMsg)
            return 'success'
        except Exception as e:
            rospy.logerr(e)
            return 'fail'
