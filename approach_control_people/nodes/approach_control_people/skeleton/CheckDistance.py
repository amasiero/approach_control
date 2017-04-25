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
		package = 'openni_tracker'
		executable = 'openni_tracker'
		# node_tracker = roslaunch.core.Node(package, executable)
		# # node_distance = roslaunch.core.Node('approach_control_people', 'pub_distance.py')
		# launch = roslaunch.scriptapi.ROSLaunch()
		# launch.start()
		# self.process_tracker = launch.launch(node_tracker)
		# # self.process_distance = launch.launch(node_distance)

	def callback(self, ndistance):
		rospy.loginfo(ndistance.data)
		if ndistance.data < 1.1:
			self.distance = 'closer'
		elif ndistance.data > 1.8:
			self.distance = 'far'
		else:
			self.distance = 'safety'


	def execute(self, userdata):
		# if self.process_tracker.is_alive():
	 	rospy.Subscriber('/torso_distance', Float64, self.callback)
	 	rospy.sleep(5)
	 	if self.distance is not None:
	 		# self.process_tracker.stop()
	 		return self.distance
		 	# else:
		 	# 	self.process_tracker.stop()
		 	# 	return 'fail'
		else:
			return 'not_ready'