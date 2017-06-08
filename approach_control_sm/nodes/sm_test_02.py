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

        smach.StateMachine.add('SET_INITIAL_POSITION', SetInitialPosition.SetInitialPosition(local='inicio'),
                               transitions={'success':'GO_ARMARIO','fail':'Done'})
        
        smach.StateMachine.add('GO_ARMARIO', GoToLocation.GoToLocation('armario'),
                               transitions={'success':'DOWN','fail':'Done'})

        smach.StateMachine.add('DOWN', Tilt.Tilt('down'),
                               transitions={'success' : 'GET_DISTANCE', 'fail' : 'Done'})

        smach.StateMachine.add('GET_DISTANCE', GetDistance.GetDistance(),
                               transitions={'safe' : 'WALK_IN', 'too_close' : 'STOP_IN', 'fail' : 'Done'})

        smach.StateMachine.add('WALK_IN', Walk.Walk(linear=0.2),
                                transitions={'walking': 'GET_DISTANCE', 'stopping' : 'WHERE_IS_BOTTLE'})

        smach.StateMachine.add('STOP_IN', Walk.Walk(),
                                transitions={'walking': 'GET_DISTANCE', 'stopping' : 'WHERE_IS_BOTTLE'})

        smach.StateMachine.add('WHERE_IS_BOTTLE', Say.Say("I let, my bottle, here."),
                                transitions={'spoke' : 'HEAD', 'mute' : 'Done'})

        smach.StateMachine.add('HEAD', GestureAction.GestureAction('head2'),
                               transitions={'success':'KITCHEN','fail':'Done'})

        smach.StateMachine.add('KITCHEN', Say.Say("Maybe in the kitchen?"),
                                transitions={'spoke' : 'GET_DISTANCE_2', 'mute' : 'Done'})

        smach.StateMachine.add('GET_DISTANCE_2', GetDistanceInvert.GetDistanceInvert(),
                               transitions={'safe' : 'WALK_OUT', 'too_close' : 'STOP_OUT', 'fail' : 'Done'})

        smach.StateMachine.add('WALK_OUT', Walk.Walk(linear=-0.2),
                                transitions={'walking': 'GET_DISTANCE_2', 'stopping' : 'FRONT'})

        smach.StateMachine.add('STOP_OUT', Walk.Walk(),
                                transitions={'walking': 'GET_DISTANCE_2', 'stopping' : 'FRONT'})

        smach.StateMachine.add('FRONT', Tilt.Tilt('front'),
                               transitions={'success' : 'GO_OBJECT', 'fail' : 'Done'})

        smach.StateMachine.add('GO_OBJECT', GoToLocation.GoToLocation('entrada_cozinha'),
                               transitions={'success':'GO_OBJECT_2','fail':'Done'})

        smach.StateMachine.add('GO_OBJECT_2', GoToLocation.GoToLocation('entrada_cozinha_2'),
                               transitions={'success':'GO_OBJECT_3','fail':'Done'})

        smach.StateMachine.add('GO_OBJECT_3', GoToLocation.GoToLocation('mesa'),
                               transitions={'success':'POINT','fail':'Done'})

        smach.StateMachine.add('POINT', GestureAction.GestureAction('point'),
                               transitions={'success':'CON_1','fail':'Done'})

        sm_con_1 = smach.Concurrence(outcomes=['success', 'fail'],
                                    default_outcome = 'fail',
                                    outcome_map = {'success' : 
                                        {'DAMMED' : 'spoke',
                                          'ANGRY_FACE' : 'success'}})
        with sm_con_1:

          smach.Concurrence.add('ANGRY_FACE', PublishFace.PublishFace('angry'))

          smach.Concurrence.add('DAMMED', Say.Say("Dammed!, It is not here too!"))
        
        smach.StateMachine.add('CON_1', sm_con_1,
                                transitions={'success':'GO_FIND_PEOPLE', 'fail':'CON_1'})

        smach.StateMachine.add('GO_FIND_PEOPLE', GoToLocation.GoToLocation('saida_cozinha'),
                               transitions={'success':'GO_FIND_PEOPLE_2','fail':'Done'})

        smach.StateMachine.add('GO_FIND_PEOPLE_2', GoToLocation.GoToLocation('saida_cozinha_2'),
                               transitions={'success':'GO_FIND_PEOPLE_3','fail':'Done'})

        smach.StateMachine.add('GO_FIND_PEOPLE_3', GoToLocation.GoToLocation('sala'),
                               transitions={'success':'SURPRISE_FACE','fail':'Done'})

        smach.StateMachine.add('SURPRISE_FACE', PublishFace.PublishFace('surprise_blured'),
                               transitions={'success':'HAPPY_FACE','fail':'Done'})

        smach.StateMachine.add('HAPPY_FACE', PublishFace.PublishFace('happy'),
                               transitions={'success':'GO_INTIMA','fail':'Done'})        

        smach.StateMachine.add('GO_INTIMA', GoToLocation.GoToLocation('sofa_p'),
                               transitions={'success':'CON_2','fail':'Done'})

        sm_con_2 = smach.Concurrence(outcomes=['success', 'fail'],
                                    default_outcome = 'fail',
                                    outcome_map = {'success' : 
                                        {'HI' : 'spoke',
                                          'HELLO_GESTURE' : 'success'}})
        with sm_con_2:
          
          smach.Concurrence.add('HELLO_GESTURE', GestureAction.GestureAction('long_hello'))

          smach.Concurrence.add('HI', Say.Say("Hi!"))

        smach.StateMachine.add('CON_2', sm_con_2,
                                transitions={'success':'QUESTION', 'fail':'CON_2'})

        smach.StateMachine.add('QUESTION', Say.Say("Could you please, help me find my salad?"),
                               transitions={'spoke' : 'YES', 'mute' : 'Done'})

        smach.StateMachine.add('YES', Recognizer.Recognizer(spec = ['Yes'], time_out = 100),
                               transitions={'Yes' : 'THANKS', 'fail' : 'YES'})

        smach.StateMachine.add('THANKS', Say.Say("Thank you, I am going outside."),
                               transitions={'spoke' : 'EXIT', 'mute' : 'Done'})

        smach.StateMachine.add('EXIT', GoToLocation.GoToLocation('saida_1'),
                               transitions={'success':'EXIT_2','fail':'Done'})

        smach.StateMachine.add('EXIT_2', GoToLocation.GoToLocation('saida_2'),
                               transitions={'success':'EXIT_3','fail':'Done'})

        smach.StateMachine.add('EXIT_3', GoToLocation.GoToLocation('fora_casa'),
                               transitions={'success':'END','fail':'Done'})

        smach.StateMachine.add('END', Say.Say("Lets do it again!"),
                               transitions={'spoke' : 'Done', 'mute' : 'Done'})


    sis = smach_ros.IntrospectionServer('Judith_StateMachineServer', sm, '/SM_JUDITH')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    rospy.init_node('test_02_sm')
    setup_sm()