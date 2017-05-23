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

        smach.StateMachine.add('RECOGNIZE', Recognizer.Recognizer(spec = ['Robot'], time_out = 100),
                               transitions={'Robot' : 'HELLO', 'fail' : 'SORRY'})

        smach.StateMachine.add('HELLO', Say.Say("Hello"),
                               transitions={'spoke' : 'START', 'mute' : 'Done'})

        smach.StateMachine.add('SORRY', Say.Say("Sorry, repeat please?"),
                                transitions={'spoke' : 'RECOGNIZE', 'mute' : 'Done'})

        smach.StateMachine.add('START', Recognizer.Recognizer(spec = ['Start'], time_out = 100),
                               transitions={'Start' : 'OK', 'fail' : 'SORRY'})

        smach.StateMachine.add('OK', Say.Say("Okay!"),
                               transitions={'spoke' : 'SET_INITIAL_POSITION', 'mute' : 'Done'})

        smach.StateMachine.add('SET_INITIAL_POSITION', SetInitialPosition.SetInitialPosition(local='test_entrada'),
                               transitions={'success':'GO_OBJECT','fail':'Done'})
        
        smach.StateMachine.add('GO_OBJECT', GoToLocation.GoToLocation('test_cozinha'),
                               transitions={'success':'GO_ARCHIVE','fail':'Done'})

        # smach.StateMachine.add('GO_CENTER', GoToLocation.GoToLocation('pa_centro'),
        #                        transitions={'success':'GO_ARCHIVE','fail':'Done'})
        
        smach.StateMachine.add('GO_ARCHIVE', GoToLocation.GoToLocation('test_corredor'),
                               transitions={'success':'GO_PESSOAL_S','fail':'Done'})
        
        smach.StateMachine.add('GO_PESSOAL_S', GoToLocation.GoToLocation('test_sala'),
                               transitions={'success':'GO_INTIMA_S','fail':'Done'})

        smach.StateMachine.add('GO_INTIMA_S', GoToLocation.GoToLocation('test_armario'),
                               transitions={'success':'GO_PORT','fail':'Done'})

        smach.StateMachine.add('GO_PORT', GoToLocation.GoToLocation('test_sofa_l'),
                               transitions={'success':'GO_PESSOAL_P','fail':'Done'})

        smach.StateMachine.add('GO_PESSOAL_P', GoToLocation.GoToLocation('test_sofa_p'),
                               transitions={'success':'GO_INTIMA_P','fail':'Done'})

        smach.StateMachine.add('GO_INTIMA_P', GoToLocation.GoToLocation('test_saida'),
                               transitions={'success':'THANKS','fail':'Done'})

        smach.StateMachine.add('THANKS', Say.Say("Thank you"),
                               transitions={'spoke' : 'Done', 'mute' : 'Done'})

    sis = smach_ros.IntrospectionServer('Judith_StateMachineServer', sm, '/SM_JUDITH')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    rospy.init_node('test_sm_robot_walking')
    setup_sm()