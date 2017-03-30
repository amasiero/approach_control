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
		
		# Preparing to start openni_tracker node
		self.package = 'openni_tracker'
		self.executable = 'openni_tracker'
		self.node = roslaunch.core.Node(self.package, self.executable)
		self.launch = roslaunch.scriptapi.ROSLaunch()
		self.launch.start()
		self.process = launch.launch(node)

	def callback(self, ndistance):
		if ndistance.data < 1.1:
			self.distance = 'closer'
		elif ndistance.data > 1.8:
			self.distance = 'far'
		else:
			self.distance = 'safety'


	def execute(self, userdata):
		if self.process.is_alive():
		 	rospy.Subscriber('/torso_distance', Float64, self.callback)
		 	rospy.sleep(5)
		 	if self.distance is not None:
		 		self.process.stop()
		 		return self.distance
		 	else:
		 		self.process.stop()
		 		return 'fail'
		else:
			return 'not_ready'