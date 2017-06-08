#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('approach_control_sm')
import smach
import smach_ros
import time

from approach_control_people import GetDistance, GetDistanceInvert
from approach_control_head import Tilt
from approach_control_movement.walk import Walk

def setup_sm():

    sm = smach.StateMachine(outcomes=['Done'])

    with sm:
        smach.StateMachine.add('FRONT', Tilt.Tilt('front'),
                               transitions={'success' : 'DOWN', 'fail' : 'Done'})

        smach.StateMachine.add('DOWN', Tilt.Tilt('down'),
                               transitions={'success' : 'GET_DISTANCE', 'fail' : 'Done'})

        smach.StateMachine.add('GET_DISTANCE', GetDistance.GetDistance(),
                               transitions={'safe' : 'WALK_IN', 'too_close' : 'STOP_IN', 'fail' : 'Done'})

        smach.StateMachine.add('WALK_IN', Walk.Walk(linear=0.2),
                                transitions={'walking': 'GET_DISTANCE', 'stopping' : 'FRONT_2'})

        smach.StateMachine.add('STOP_IN', Walk.Walk(),
                                transitions={'walking': 'GET_DISTANCE', 'stopping' : 'FRONT_2'})

        smach.StateMachine.add('FRONT_2', Tilt.Tilt('front'),
                               transitions={'success' : 'DOWN_2', 'fail' : 'Done'})

        smach.StateMachine.add('DOWN_2', Tilt.Tilt('down'),
                               transitions={'success' : 'GET_DISTANCE_2', 'fail' : 'Done'})

        smach.StateMachine.add('GET_DISTANCE_2', GetDistanceInvert.GetDistanceInvert(),
                               transitions={'safe' : 'WALK_OUT', 'too_far' : 'STOP_OUT', 'fail' : 'Done'})

        smach.StateMachine.add('WALK_OUT', Walk.Walk(linear=-0.2),
                                transitions={'walking': 'GET_DISTANCE_2', 'stopping' : 'FRONT_3'})

        smach.StateMachine.add('STOP_OUT', Walk.Walk(),
                                transitions={'walking': 'GET_DISTANCE_2', 'stopping' : 'FRONT_3'})

        smach.StateMachine.add('FRONT_3', Tilt.Tilt('front'),
                               transitions={'success' : 'Done', 'fail' : 'Done'})

        # smach.StateMachine.add('DOWN', Tilt.Tilt('down'),
        #                        transitions={'success' : 'CHECK_DISTANCE', 'fail' : 'Done'})

        # smach.StateMachine.add('GET_DISTANCE', GetDistance.GetDistance(),
        #                        transitions={'success' : 'CHECK_DISTANCE', 'fail' : 'Done'})

        # smach.StateMachine.add('CHECK_DISTANCE', DistanceVerification.DistanceVerification(),
        #                        transitions={'intimate' : 'INTIMATE', 'personal' : 'PERSONAL', 'social' : 'SOCIAL', 'public' : 'PUBLIC', 'not_ready' : 'NOT_READY', 'fail' : 'Done'})

        # smach.StateMachine.add('INTIMATE', DistanceDisplay.DistanceDisplay('intimate'),
        #                        transitions={'published' : 'GET_DISTANCE', 'not' : 'GET_DISTANCE'})
        
        # smach.StateMachine.add('PERSONAL', DistanceDisplay.DistanceDisplay('personal'),
        #                        transitions={'published' : 'GET_DISTANCE', 'not' : 'GET_DISTANCE'})
        
        # smach.StateMachine.add('SOCIAL', DistanceDisplay.DistanceDisplay('social'),
        #                        transitions={'published' : 'GET_DISTANCE', 'not' : 'GET_DISTANCE'})
        
        # smach.StateMachine.add('PUBLIC', DistanceDisplay.DistanceDisplay('public'),
        #                        transitions={'published' : 'GET_DISTANCE', 'not' : 'GET_DISTANCE'})

        # smach.StateMachine.add('NOT_READY', DistanceDisplay.DistanceDisplay('not ready'),
        #                        transitions={'published' : 'GET_DISTANCE', 'not' : 'GET_DISTANCE'})

      
    sis = smach_ros.IntrospectionServer('Judith_StateMachineServer', sm, '/SM_JUDITH')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    rospy.init_node('test_01_sm')
    setup_sm()