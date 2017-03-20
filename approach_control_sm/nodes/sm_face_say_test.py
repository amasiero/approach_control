#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('approach_control_sm')
import smach
import smach_ros
import time

from approach_control_voice import Say
from approach_control_people.faces import FaceFinder, PersonFaceCapture

def setup_sm():

	sm = smach.StateMachine(outcomes=['done'])

	with sm:

		smach.StateMachine.add('FINDER', FaceFinder.FaceFinder(),
								transitions={'one_face' : 'SAY_ONE', 'faces' : 'SAY_MULTI', 'searching' : 'FINDER', 'fail' : 'done'})
	
		smach.StateMachine.add('SAY_ONE', Say.Say('Please, face me so I can record you.'),
								transitions={'spoke' : 'RECORD_FACE', 'mute' : 'done'})

		smach.StateMachine.add('SAY_MULTI', Say.Say('I talk to you, when you are alone.'),
								transitions={'spoke' : 'done', 'mute' : 'done'})

		smach.StateMachine.add('RECORD_FACE', PersonFaceCapture.PersonFaceCapture(),
								transitions={'success' : 'THANKS_MSG', 'in_progress' : 'RECORD_FACE', 'fail' : 'done'})

		smach.StateMachine.add('THANKS_MSG', Say.Say('Now I will remmenber you.'),
								transitions={'spoke' : 'done', 'mute' : 'done'})

	sis = smach_ros.IntrospectionServer('Judith_StateMachineServer', sm, '/SM_JUDITH')
	sis.start()

	outcome = sm.execute()

	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	rospy.init_node('sm_face_say_test')
	setup_sm()