#!/usr/bin/env python

import rospy
import smach
from openni import *


class DistanceVerification(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['intimate', 'personal', 'social', 'public', 'not_ready', 'fail'])
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
        center_y = depth_map.height / 2

        distance_in_cm =  depth_map[center_x, center_y] / 10.0

        if distance_in_cm < 50:
            return 'intimate'
        elif distance_in_cm >= 50 and distance_in_cm < 120:
            return 'personal'
        elif distance_in_cm >= 120 and distance_in_cm < 360:
            return 'social'
        elif distance_in_cm >= 360:
            return 'public'
        else:
            return 'not_ready'
