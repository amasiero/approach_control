#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('approach_control_sm')
import smach
import smach_ros
import time

from approach_control_speech import Say
from approach_control_people.skeleton import GetSkeleton

def setup_sm():

    sm = smach.StateMachine(outcomes=['Done'])

    with sm:

        smach.StateMachine.add('HELLO', Say.Say("Hello"),
                               transitions={'spoke' : 'SKELETON', 'mute' : 'Done'})

<<<<<<< HEAD
        smach.StateMachine.add('SKELETON', GetSkeleton.GetSkeleton(),
=======
        smach.StateMachine.add('SKELETON', GestureAction.GestureAction('short'),
>>>>>>> 18e79008792b4c6af3b2a2f34c8e0cc0f824e13a
                               transitions={'success':'SKELETON'})

    sis = smach_ros.IntrospectionServer('Judith_StateMachineServer', sm, '/SM_JUDITH')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    rospy.init_node('test_sm_robot_walking')
    setup_sm()
