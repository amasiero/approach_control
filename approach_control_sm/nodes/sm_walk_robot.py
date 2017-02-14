#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('approach_control_sm')
import smach
import smach_ros
import time

def main():
	rospy.init_node('approach_control_walk_sm')

if __name__ == '__main__':
	main()