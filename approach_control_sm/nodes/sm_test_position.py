#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('approach_control_sm')
import smach
import smach_ros
import time

from approach_control_speech import Say, Recognizer
from approach_control_people.skeleton import CheckDistance
from approach_control_navigation import GoToLocation, SetInitialPosition
from approach_control_manipulator import GestureAction
from approach_control_robot_face import PublishFace

def setup_sm():

    sm = smach.StateMachine(outcomes=['Done'])

    with sm:

        smach.StateMachine.add('HELLO', Say.Say("Hello"),
                               transitions={'spoke' : 'SET_INITIAL_POSITION', 'mute' : 'Done'})

        smach.StateMachine.add('SET_INITIAL_POSITION', SetInitialPosition.SetInitialPosition(local='test_entrada'),
                               transitions={'success':'Done','fail':'Done'})
        
    sis = smach_ros.IntrospectionServer('Judith_StateMachineServer', sm, '/SM_JUDITH')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    rospy.init_node('test_sm_robot_walking')
    setup_sm()