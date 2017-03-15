#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('approach_control_sm')
import smach
import smach_ros
import time

from approach_control_voice import Say
from approach_control_people.faces import FaceFinder

def setup_sm():

	sm = smach.StateMachine(outcomes=['Done'])

	with sm:

		smach.StateMachine.add('WALK', Walk.Walk(0.2),
								transitions={'walking' : 'CHECK_DISTANCE', 'stopping' : 'STOP'})
	
		smach.StateMachine.add('CHECK_DISTANCE', Laser.Laser(),
								transitions={'closerFront' : 'STOP', 'closerRight' : 'STOP', 'closerLeft' : 'STOP', 'far' : 'WALK'})

		smach.StateMachine.add('STOP', Walk.Walk(),
								transitions={'walking' : 'Done', 'stopping' : 'Done'})

	sis = smach_ros.IntrospectionServer('Judith_StateMachineServer', sm, '/SM_JUDITH')
	sis.start()

	outcome = sm.execute()

	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	rospy.init_node('test_sm_robot_walking')
	setup_sm()