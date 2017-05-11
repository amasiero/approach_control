#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('approach_control_sm')
import smach
import smach_ros
import time

from approach_control_speech import Say, Recognizer
from approach_control_people.skeleton import CheckDistance
from approach_control_navigation import GoToLocation, SetInitialPosition
from approach_control_manipulator import GestureExecute
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

        smach.StateMachine.add('SET_INITIAL_POSITION', SetInitialPosition.SetInitialPosition(local='inicio'),
                               transitions={'success':'GO_OBJECT','fail':'Done'})
        
        smach.StateMachine.add('GO_OBJECT', GoToLocation.GoToLocation('mesa_objeto'),
                               transitions={'success':'POINT','fail':'Done'})

        smach.StateMachine.add('POINT', GestureExecute.GestureExecute('point'),
                               transitions={'success':'ANGRY_FACE','fail':'Done'})
        
        smach.StateMachine.add('ANGRY_FACE', PublishFace.PublishFace('angry'),
                               transitions={'success':'GO_FIND_PEOPLE','fail':'Done'})        
        
        smach.StateMachine.add('GO_FIND_PEOPLE', GoToLocation.GoToLocation('achou_pessoa'),
                               transitions={'success':'SURPRISE_FACE','fail':'Done'})

        smach.StateMachine.add('SURPRISE_FACE', PublishFace.PublishFace('surprise'),
                               transitions={'success':'HAPPY_FACE','fail':'Done'})        

        smach.StateMachine.add('HAPPY_FACE', PublishFace.PublishFace('happy'),
                               transitions={'success':'GO_INTIMA','fail':'Done'})        

        smach.StateMachine.add('GO_INTIMA', GoToLocation.GoToLocation('intima'),
                               transitions={'success':'HI','fail':'Done'})

        smach.StateMachine.add('HI', Say.Say("Hi!"),
                               transitions={'spoke' : 'HELLO_GESTURE', 'mute' : 'Done'})

        smach.StateMachine.add('HELLO_GESTURE', GestureExecute.GestureExecute('short'),
                               transitions={'success':'QUESTION','fail':'Done'})

        smach.StateMachine.add('QUESTION', Say.Say("Could you please help me find my cap?"),
                               transitions={'spoke' : 'YES', 'mute' : 'Done'})

        smach.StateMachine.add('YES', Recognizer.Recognizer(spec = ['Yes'], time_out = 100),
                               transitions={'Yes' : 'THANKS', 'fail' : 'YES'})

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