#!/usr/bin/env python

import smach
import rospy
import pyglet
import math
import numpy as np
from dynamixel_msgs.msg import JointState

class CloseGripper(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['success', 'in_progress', 'fail'])