#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseStampted
import yaml
import os
import os.path


class GetLocals:
	def __init__(self):

		rospy.Subscriber('move_base_simple/goal', PoseStampted, self.goal_callback)
		print('\nPLEASE, SEND A GOAL WITH NAV 2D GOAL USING GRAPHIC USER INTERFACE TO SAVE A POINT!\n')

	def goal_callback(self, data):

		self.fname = os.path.expanduser('~') + '/catkin_ws/src/approach_control/approach_control_config/config/locals.yaml'
		stream = open(self.fname, 'r')
		self.data = yaml.load(stream)
		self.keys = self.data.keys()
		local = raw_input('Please, write the location for this point (if doesnt exist it will be create): \n options: ' + str(self.keys))

		if [x for x in self.keys if x == local]:
			self.data[local] = [[data.pose.position.x, data.pose.position.y], [0.0, 0.0, data.pose.orientation.z, data.pose.orientation.w]]
			with open(self.fname, 'w') as yaml_file:
				yaml_file.write(yaml.dump(self.data, default_flow_style = False))
			rospy.loginfo('Point Saved!')
			rospy.loginfo('\nPLEASE, SEND A GOAL WITH NAV 2D GOAL USING GRAPHIC USER INTERFACE TO SAVE A POINT!\n')
		else:
			c = raw_input('Save as a new place? (Y/N)')
			if c.lower() == 'y':
				self.data[local] = [[data.pose.position.x, data.pose.position.y], [0.0, 0.0, data.pose.orientation.z, data.pose.orientation.w]]
				with open(self.fname, 'w') as yaml_file:
					yaml_file.write(yaml.dump(self.data, default_flow_style = False))
				rospy.loginfo('Point Saved!')
				rospy.loginfo('\nPLEASE, SEND A GOAL WITH NAV 2D GOAL USING GRAPHIC USER INTERFACE TO SAVE A POINT!\n')
			else:
				rospy.logerr('Point not Saved!')
				rospy.loginfo('\nPLEASE, SEND A GOAL WITH NAV 2D GOAL USING GRAPHIC USER INTERFACE TO SAVE A POINT!\n')


if __name__ == '__main__':
	GetLocals()
	rospy.init_node('getlocals', anonymous = True)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo('Shutting down!')
	