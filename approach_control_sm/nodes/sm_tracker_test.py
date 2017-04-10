#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('approach_control_sm')
import smach
import smach_ros
import time

from approach_control_speech import Say, Recognizer
from approach_control_people.skeleton import CheckDistance    

def setup_sm():

    sm = smach.StateMachine(outcomes=['Done'])

    with sm:

        smach.StateMachine.add('RECOGNIZE', Recognizer.Recognizer(spec = ['Robot'], time_out = 100),
                               transitions={'Robot' : 'HELLO', 'fail' : 'SORRY'})

        smach.StateMachine.add('HELLO', Say.Say("Hello"),
                               transitions={'spoke' : 'GET_DISTANCE', 'mute' : 'Done'})

        smach.StateMachine.add('SORRY', Say.Say("Sorry, repeat please?"),
                                transitions={'spoke' : 'RECOGNIZE', 'mute' : 'Done'})

        smach.StateMachine.add('GET_DISTANCE', CheckDistance.CheckDistance(),
                               transitions={'safety' : 'SAFETY', 'far' : 'FAR', 'closer' : 'CLOSER', 'not_ready' : 'NOT_READY', 'fail' : 'Done'})

        smach.StateMachine.add('SAFETY', Say.Say("We are in a good distance."),
                               transitions={'spoke' : 'RECOGNIZE', 'mute' : 'Done'})

        smach.StateMachine.add('FAR', Say.Say("You are so far, came closer to me."),
                               transitions={'spoke' : 'RECOGNIZE', 'mute' : 'Done'})

        smach.StateMachine.add('CLOSER', Say.Say("I'm closer to you."),
                               transitions={'spoke' : 'RECOGNIZE', 'mute' : 'Done'})

        smach.StateMachine.add('NOT_READY', Say.Say("Wait a minute, please."),
                               transitions={'spoke' : 'GET_DISTANCE', 'mute' : 'Done'})

    sis = smach_ros.IntrospectionServer('Judith_StateMachineServer', sm, '/SM_JUDITH')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    rospy.init_node('test_sm_robot_walking')
    setup_sm()