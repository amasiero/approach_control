#!/usr/bin/env python

import rospy
import rosparam
import actionlib
import os.path
import smach
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

class GoToLocation(smach.State):

    def __init__(self, local):
        smach.State.__init__(self, outcomes = ['success', 'fail'])

        params = rosparam.load_file(os.path.expanduser('~') + '/catkin_ws/src/approach_control/approach_control_config/config/locals.yaml')

        keys = params[0][0].keys()

        for key in keys:
            if key == local:
                local_values = params[0][0][key]

        # Clear Costmaps
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.srv_clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty, persistent = True)


        # Get an action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Define the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = local_values[0][0]
        self.goal.target_pose.pose.position.y = local_values[0][1]
        self.goal.target_pose.pose.orientation.z = local_values[1][2]
        self.goal.target_pose.pose.orientation.w = local_values[1][3]


    def execute(self, userdata):
        self.srv_clear_costmap()
        rospy.sleep(0.2)
        self.client.send_goal(self.goal)
        self.client.wait_for_result()

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            return 'success'
        else:
            rospy.logerr('GOAL STATUS: %s', str(actionlib.GoalStatus()))
            return 'fail'