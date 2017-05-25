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
        
        smach.StateMachine.add('GO_OBJECT', GoToLocation.GoToLocation('test_mesa'),
                               transitions={'success':'POINT','fail':'Done'})

        smach.StateMachine.add('POINT', GestureAction.GestureAction('point'),
                               transitions={'success':'ANGRY_FACE','fail':'Done'})
        
        smach.StateMachine.add('ANGRY_FACE', PublishFace.PublishFace('angry'),
                               transitions={'success':'GO_FIND_PEOPLE','fail':'Done'})        
        
        smach.StateMachine.add('GO_FIND_PEOPLE', GoToLocation.GoToLocation('test_sala'),
                               transitions={'success':'SURPRISE_FACE','fail':'Done'})

        smach.StateMachine.add('SURPRISE_FACE', PublishFace.PublishFace('surprise'),
                               transitions={'success':'HAPPY_FACE','fail':'Done'})

        # smach.StateMachine.add('SURPRISE_FACE_2', PublishFace.PublishFace('surprise'),
        #                        transitions={'success':'SURPRISE_FACE_3','fail':'Done'}) 

        # smach.StateMachine.add('SURPRISE_FACE_3', PublishFace.PublishFace('surprise'),
        #                        transitions={'success':'HAPPY_FACE','fail':'Done'})         

        smach.StateMachine.add('HAPPY_FACE', PublishFace.PublishFace('happy'),
                               transitions={'success':'GO_INTIMA','fail':'Done'})        

        smach.StateMachine.add('GO_INTIMA', GoToLocation.GoToLocation('test_sofa_p'),
                               transitions={'success':'CON','fail':'Done'})

        sm_con = smach.Concurrence(outcomes=['success', 'fail'],
                                    default_outcome = 'fail',
                                    outcome_map = {'success' : 
                                        {'HI' : 'spoke',
                                          'HELLO_GESTURE' : 'success'}})

        with sm_con:
          
          smach.Concurrence.add('HELLO_GESTURE', GestureAction.GestureAction('short'))

          smach.Concurrence.add('HI', Say.Say("Hi!"))

        smach.StateMachine.add('CON', sm_con,
                                transitions={'success':'QUESTION', 'fail':'CON'})

        smach.StateMachine.add('QUESTION', Say.Say("Could you please help me find my cap?"),
                               transitions={'spoke' : 'YES', 'mute' : 'Done'})

        smach.StateMachine.add('YES', Recognizer.Recognizer(spec = ['Yes'], time_out = 100),
                               transitions={'Yes' : 'THANKS', 'fail' : 'YES'})

        smach.StateMachine.add('THANKS', Say.Say("Thank you"),
                               transitions={'spoke' : 'EXIT', 'mute' : 'Done'})

        smach.StateMachine.add('EXIT', GoToLocation.GoToLocation('test_saida'),
                               transitions={'success':'Done','fail':'Done'})

    sis = smach_ros.IntrospectionServer('Judith_StateMachineServer', sm, '/SM_JUDITH')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    rospy.init_node('test_01_sm')
    setup_sm()