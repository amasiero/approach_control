#!/usr/bin/env python

import rospy
import rosparam
import smach
import os.path
from geometry_msgs.msg import PoseWithCovarianceStamped

class SetInitialPosition(smach.State):

    def __init__(self, local = 'Initial Position', var = 0.05):
        smach.State.__init__(self, outcomes = ['success', 'fail'])

        params = rosparam.load_file(os.path.expanduser('~') + '/catkin_ws/src/approach_control/approach_control_config/config/locals.yaml')
        self.initPose = rospy.Publisher('/initialPose', PoseWithCovarianceStamped, queue_size = 0)

        rospy.logwarn(params)

        rospy.logerr('local ==>' + local)
        if rospy.has_param(local):
            local_values =  rospy.get_param(rospy.search_param(local))

            # Define the initial position
            self.initialPositionMsg = PoseWithCovarianceStamped()
            self.initialPositionMsg.pose.position.x = local_values[0][0]
            self.initialPositionMsg.pose.position.y = local_values[0][1]
            self.initialPositionMsg.pose.orientation.w = local_values[1][3]
            self.initialPositionMsg.pose.orientation.z = local_values[1][2]


            for x in range(0, 35, 5):
                self.initialPositionMsg.pose.covariance[x] = var
        else:
            rospy.logerr('Param not found!')

        

    def execute(self, userdata):
        try:
            self.initPose.publish(self.initialPositionMsg)
            return 'success'
        except:
            return 'fail'
