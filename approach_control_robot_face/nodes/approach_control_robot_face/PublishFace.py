#!/usr/bon/env python

import rospy
import smach
from std_msgs.msg import String

class PublishFace(smach.State):

    def __init__(self, face='neutral'):
        smach.State.__init__(self, outcomes=['success', 'fail'])
        self.face = face
        self.publish_face = rospy.Publisher('face', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo('Publishing a face...')
        rospy.sleep(0.1)

        if self.face is not None:
            self.publish_face.publish(self.face)
            if self.face == 'surprise_blured':
                rospy.sleep(1)
            return 'success'
        else:
            rospy.logerr('Face not found...')
            return 'fail'