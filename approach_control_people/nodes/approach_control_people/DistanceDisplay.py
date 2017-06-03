#!/usr/bin/env python

import rospy
import smach


class DistanceDisplay(smach.State):

    def __init__(self, distance=''):
        smach.State.__init__(self, outcomes=['published', 'not'])
        self.distance = distance

    def execute(self, userdata):

        rospy.loginfo('distance published %s' % self.distance)

        if self.distance is not '':
            return 'published'
        else:
            return 'not'
