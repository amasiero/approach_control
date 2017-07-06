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
from approach_control_people import GetDistance, GetDistanceInvert
from approach_control_head import Tilt
from approach_control_movement.walk import Walk

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

        smach.StateMachine.add('SET_INITIAL_POSITION', SetInitialPosition.SetInitialPosition(local='jardim'),
                               transitions={'success':'GO_ARMARIO','fail':'Done'})

        smach.StateMachine.add('GO_ARMARIO', GoToLocation.GoToLocation('armario'),
                               transitions={'success':'QUESTION_FACE','fail':'Done'})

        smach.StateMachine.add('QUESTION_FACE', PublishFace.PublishFace('question'),
                               transitions={'success':'HEAD','fail':'Done'})

        smach.StateMachine.add('HEAD', GestureAction.GestureAction('head2'),
                       transitions={'success':'SAD_FACE','fail':'Done'})

        smach.StateMachine.add('SAD_FACE', PublishFace.PublishFace('sad'),
                               transitions={'success':'WHERE_IS_BOTTLE','fail':'Done'})

        smach.StateMachine.add('WHERE_IS_BOTTLE', Say.Say("I left my bottle here."),
                                transitions={'spoke' : 'HALL', 'mute' : 'Done'})

        smach.StateMachine.add('HALL', Say.Say("Maybe did I let in the hall?"),
                                transitions={'spoke' : 'TABLE', 'mute' : 'Done'})

        smach.StateMachine.add('TABLE', GoToLocation.GoToLocation('mesa'),
                               transitions={'success':'CON_1','fail':'Done'})

        sm_con_1 = smach.Concurrence(outcomes=['success', 'fail'],
                                    default_outcome = 'fail',
                                    outcome_map = {'success' :
                                        {'DAMMED' : 'spoke',
                                          'ANGRY_FACE' : 'success'}})
        with sm_con_1:

          smach.Concurrence.add('ANGRY_FACE', PublishFace.PublishFace('sad'))

          smach.Concurrence.add('DAMMED', Say.Say("Gosh! It is not here too!"))

        smach.StateMachine.add('CON_1', sm_con_1,
                                transitions={'success':'SOFA_F', 'fail':'CON_1'})

        smach.StateMachine.add('SOFA_F', GoToLocation.GoToLocation('pessoa_longe'),
                               transitions={'success':'SURPRISE_FACE','fail':'Done'})

        smach.StateMachine.add('SURPRISE_FACE', PublishFace.PublishFace('surprise_blured'),
                               transitions={'success':'HAPPY_FACE','fail':'Done'})

        smach.StateMachine.add('HAPPY_FACE', PublishFace.PublishFace('happy'),
                               transitions={'success':'SOFA_C','fail':'Done'})

        smach.StateMachine.add('SOFA_C', GoToLocation.GoToLocation('pessoa_perto'),
                       transitions={'success':'CON_2','fail':'Done'})

        sm_con_2 = smach.Concurrence(outcomes=['success', 'fail'],
                                    default_outcome = 'fail',
                                    outcome_map = {'success' :
                                        {'HI' : 'spoke',
                                          'HELLO_GESTURE' : 'success'}})
        with sm_con_2:

          smach.Concurrence.add('HELLO_GESTURE', GestureAction.GestureAction('point'))

          smach.Concurrence.add('HI', Say.Say("Hi!"))

        smach.StateMachine.add('CON_2', sm_con_2,
                                transitions={'success':'QUESTION', 'fail':'CON_2'})

        smach.StateMachine.add('QUESTION', Say.Say("Could you please help me find my bottle?"),
                               transitions={'spoke' : 'YES', 'mute' : 'Done'})

        smach.StateMachine.add('YES', Recognizer.Recognizer(spec = ['Yes'], time_out = 100),
                               transitions={'Yes' : 'FOLLOW', 'fail' : 'YES'})

        smach.StateMachine.add('FOLLOW', Say.Say("Please follow me"),
                               transitions={'spoke' : 'EXIT', 'mute' : 'Done'})

        smach.StateMachine.add('EXIT', GoToLocation.GoToLocation('saida'),
                               transitions={'success':'THANKS','fail':'Done'})

        smach.StateMachine.add('THANKS', Say.Say("Thanks for your cooperation"),
                               transitions={'spoke' : 'END', 'mute' : 'Done'})

        smach.StateMachine.add('BACK_HOME', GoToLocation.GoToLocation('jardim'),
                               transitions={'success':'Done','fail':'Done'})

    sis = smach_ros.IntrospectionServer('Judith_StateMachineServer', sm, '/SM_JUDITH')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    rospy.init_node('test_02_sm')
    setup_sm()
