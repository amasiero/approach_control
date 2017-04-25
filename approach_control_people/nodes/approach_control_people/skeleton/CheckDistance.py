#!/usr/bin/env python

import rospy
import roslaunch
import smach
import smach_ros
from std_msgs.msg import Float64

class CheckDistance(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['safety','far', 'closer', 'not_ready', 'fail'])
		self.distance = None
		
		
	def callback(self, ndistance):
		rospy.loginfo(ndistance.data)
		if ndistance.data < 1.1:
			self.distance = 'closer'
		elif ndistance.data > 1.8:
			self.distance = 'far'
		else:
			self.distance = 'safety'


	def execute(self, userdata):
		
		package = 'openni_tracker'
		executable = 'openni_tracker'
		
		node_tracker = roslaunch.core.Node(package, executable)
		
		launch = roslaunch.scriptapi.ROSLaunch()
		launch.start()
		
		process_tracker = launch.launch(node_tracker)
		
		rospy.Subscriber('/torso_distance', Float64, self.callback)
	 	rospy.sleep(5)
	 	
	 	if self.distance is not None:
	 		return self.distance
		else:
			return 'not_ready'