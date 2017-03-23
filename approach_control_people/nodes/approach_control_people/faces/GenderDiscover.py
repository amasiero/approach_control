#!/usr/bin/env python

import cv2.cv
import imutils
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from approach_control_people.faces import *
from approach_control_people.faces.load_database import load_female_db as female_db
from approach_control_people.faces.load_database import load_male_db as male_db


class GenderDiscover(smach.State):
	
	def __init__(self):
		smach.State.__init__(self, outcomes=['woman', 'man', 'fail'])
		self.face_cascade = cv2.CascadeClassifier('/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml')
		self.bridge = CvBridge()
		
		self.GRID = 16
		self.SIZE = 128

		self.m = Map.Map('Regions')
		self.m.MakeRegularCluster(self.SIZE, self.SIZE, self.GRID, self.GRID)
		self.m.MakeRegions()

		