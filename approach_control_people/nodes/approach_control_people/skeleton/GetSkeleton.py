#!/usr/bin/env python

import rospy
import smach
import math
from openni import *

class GetSkeleton(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['far', 'close', 'fail'])
        self.pose_to_use = ''

        self.ctx = Context()
        self.ctx.init()

        self.user = UserGenerator()
        self.user.create(ctx)

        self.skel_cap = self.user.skeleton_cap
        self.pose_cap = self.user.pose_detection_cap

        self.user.register_user_cb(self.new_user, self.lost_user)
        self.pose_cap.register_pose_detected_cb(self.pose_detected)
        self.skel_cap.register_c_start_cb(self.calibration_start)
        self.skel_cap.register_c_complete_cb(self.calibration_complete)

        self.skel_cap.set_profile(SKEL_PROFILE_ALL)

        self.ctx.start_generating_all()
        print "0/4 Starting to detect users. Press Ctrl-C to exit."

    def new_user(self, src, id):
        print "1/4 User {} detected. Looking for pose...".format(id)
        self.pose_cap.start_detection(self.pose_to_use, id)

    def pose_detected(self, src, pose, id):
        print "2/4 Detected pose {} on user {}. Requesting calibration...".format(pose, id)
        self.pose_cap.stop_detection(id)
        self.skel_cap.request_calibration(id, True)

    def calibration_start(self, src, id):
        print "3/4 Calibration started for user {}.".format(id)

    def calibration_complete(self, src, id, status):
        if status == CALIBRATION_STATUS_OK:
            print "4/4 User {} calibrated successfully! Starting to track.".format(id)
            self.skel_cap.start_tracking(id)
        else:
            print "ERR User {} failed to calibrate. Restarting process.".format(id)
            self.new_user(self.user, id)

    def lost_user(self, src, id):
        print "--- User {} lost.".format(id)

    def execute(self, userdata):
        self.ctx.wait_and_update_all()

        for id in self.user.users:
            if self.skel_cap.is_tracking(id):
                head = self.skel_cap.get_joint_position(id, SKEL_HEAD)
                print "  {}: head at ({loc[0]}, {loc[1]}, {loc[2]}) [{conf}]" .format(id, loc=head.point, conf=head.confidence)

        return 'success'
