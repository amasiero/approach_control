#!/usr/bin/env python

import math
import numpy as np
import scipy.optimize

class Arm3Link:

	def __init__(self, q = None, q0 = None, L = None):
		"""
		Set up the basic parameters of the arm.
		All lists are in order [shoulder, elbow, wrist].

		:param list q: the initial joint angles of the arm
		:param list q0: the default (resting state) joint configuration
		:param list L: the arm segment lengths
		"""

		# initial joint angles
		# ternary operator python (if_test_is_false, if_test_is_true)[test]
		self.q = (q, [math.pi / 2, -math.pi / 2, 0, math.pi / 2])[q is None]
		
		self.q0 = (q0, [math.pi / 4, math.pi / 4, 0, math.pi / 2])[q0 is None]
		
		self.L = (L, np.array([1, 1, 1]))[L is None]
		
		self.max_angles = [math.pi, math.pi, math.pi / 4, math.pi / 4]
		self.min_angles = [0, 0, -math.pi / 4, -math.pi / 4]

	def get_xy(self, q = None):
		"""
		Returns the corresponding hand xyz coordinates for 
		a given set of joint angle values [shoulder, elbow, wrist],
		and the above defined arm segment lengths, L.

		:parm list q: the list of current joint angles
		:returns list: the [x, y, z] position of the arm
		"""
		if q is None: q = self.q

		x = (self.L[0] * np.cos(q[0]) + \
			 self.L[1] * np.cos(q[0] + q[1]) + \
			 self.L[2] * 
			)