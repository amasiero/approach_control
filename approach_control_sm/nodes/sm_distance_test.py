#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('approach_control_sm')
import smach
import smach_ros
import time

from approach_control_people import GetDistance, DistanceVerification, DistanceDisplay
from approach_control_head import Tilt

def setup_sm():

    sm = smach.StateMachine(outcomes=['Done'])

    with sm:
        smach.StateMachine.add('FRONT', Tilt.Tilt('front'),
                               transitions={'success' : 'DOWN', 'fail' : 'Done'})

        smach.StateMachine.add('DOWN', Tilt.Tilt('down'),
                               transitions={'success' : 'CHECK_DISTANCE', 'fail' : 'Done'})

        smach.StateMachine.add('GET_DISTANCE', GetDistance.GetDistance(),
                               transitions={'success' : 'CHECK_DISTANCE', 'fail' : 'Done'})

        smach.StateMachine.add('CHECK_DISTANCE', DistanceVerification.DistanceVerification(),
                               transitions={'intimate' : 'INTIMATE', 'personal' : 'PERSONAL', 'social' : 'SOCIAL', 'public' : 'PUBLIC', 'not_ready' : 'NOT_READY', 'fail' : 'Done'})

        smach.StateMachine.add('INTIMATE', DistanceDisplay.DistanceDisplay('intimate'),
                               transitions={'published' : 'GET_DISTANCE', 'not' : 'GET_DISTANCE'})
        
        smach.StateMachine.add('PERSONAL', DistanceDisplay.DistanceDisplay('personal'),
                               transitions={'published' : 'GET_DISTANCE', 'not' : 'GET_DISTANCE'})
        
        smach.StateMachine.add('SOCIAL', DistanceDisplay.DistanceDisplay('social'),
                               transitions={'published' : 'GET_DISTANCE', 'not' : 'GET_DISTANCE'})
        
        smach.StateMachine.add('PUBLIC', DistanceDisplay.DistanceDisplay('public'),
                               transitions={'published' : 'GET_DISTANCE', 'not' : 'GET_DISTANCE'})

        smach.StateMachine.add('NOT_READY', DistanceDisplay.DistanceDisplay('not ready'),
                               transitions={'published' : 'GET_DISTANCE', 'not' : 'GET_DISTANCE'})

      
    sis = smach_ros.IntrospectionServer('Judith_StateMachineServer', sm, '/SM_JUDITH')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    rospy.init_node('test_01_sm')
    setup_sm()