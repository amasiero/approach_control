#!/usr/bin/env python

import rospy
import smach
import math
from openni import *



class GetDistanceInvert(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['safe', 'too_far', 'fail'])
        self.ctx = Context()
        self.ctx.init()

        self.depth = DepthGenerator()
        self.depth.create(self.ctx)

        self.depth.set_resolution_preset(RES_VGA)
        self.depth.fps = 30

        self.ctx.start_generating_all()

    def execute(self, userdata):

        nRetVal = self.ctx.wait_one_update_all(self.depth)

        depth_map = self.depth.map

        center_x = depth_map.width / 2
        y = depth_map.height - 1

        distance_in_cm =  depth_map[center_x, y] / 10.0

        real_distance = 0
        distance_stop_in = 100
        distance_stop_out = 135

        if distance_in_cm > 135:
            real_distance = math.sqrt(math.pow(distance_in_cm, 2) - math.pow(135, 2))
            distance_stop_in = 40
            distance_stop_out = 90
        else:
            real_distance = distance_in_cm


        rospy.logwarn(distance_in_cm)
        rospy.logwarn(real_distance)
        rospy.logwarn(distance_stop_in)
        rospy.logwarn(distance_stop_out)

        if real_distance <= distance_stop_out and real_distance > distance_stop_in:
            return 'too_far'
        elif real_distance < distance_stop_in:
            return 'safe'
        else:
            return 'fail'
