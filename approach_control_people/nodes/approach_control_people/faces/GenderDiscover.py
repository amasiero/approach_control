#!/usr/bin/env python

import cv2.cv
import imutils
import time
import smach
import smach_ros
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from approach_control_people.faces.Map import Map
from approach_control_people.faces.ULBP import ULBP
from approach_control_people.faces.lbp_utils import W, Authentify
from approach_control_people.faces.load_database import load_female_db as female_db
from approach_control_people.faces.load_database import load_male_db as male_db


class GenderDiscover(smach.State):
	
	def __init__(self):
		smach.State.__init__(self, outcomes=['woman', 'man', 'fail'])
		self.face_cascade = cv2.CascadeClassifier('/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml')
		self.bridge = CvBridge()
		
		self.GRID = 16
		self.SIZE = 128

		self.m = Map('Regions')
		self.m.MakeRegularCluster(self.SIZE, self.SIZE, self.GRID, self.GRID)
		self.m.MakeRegions()

		self.ulbp_face = ULBP(self.m)
		self.ulbp_female = ULBP(self.m)
		self.ulbp_male = ULBP(self.m)

		self.ulbp_male.MakePattern(male_db())
		self.ulbp_male.MakeHistogram()

		self.ulbp_female.MakePattern(female_db())
		self.ulbp_female.MakeHistogram()

		self.gender = None


	def is_woman(self, img):
		self.ulbp_face.MakePattern(img)
		self.ulbp_face.MakeHistogram()

		return Authentify(ulbp_face.histogram, ulbp_female.histogram, ulbp_male.histogram, W) > 20.0

	def callback(self, data):

		try:
			image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
		except CvBridgeError as e:
			rospy.logerr(e)

		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		roi_gray = None
		
		faces = self.face_cascade.detectMultScale(
			gray,
			scaleFactor = 1.1,
			minNeighbors = 10,
			minSize = (100,100),
			flags = cv2.cv.CV_HAAR_SCALE_IMAGE
		)

		for (x, y, w, h) in faces:
			x1 = x + int(w * .1)
			x2 = x1 + int(w * .8)

			y1 = y + int(h * .2)
			y2 = y1 + int(h * .8)

			roi_gray = cv2.resize(gray[y1:y2, x1:x2], (128, 128))

			self.gender = 'man'
			if is_woman(roi_gray):
				self.gender = 'woman'


	def execute(self, userdata):
		rospy.Subscriber('/image_raw', Image, self.callback)
		rospy.sleep(5)
		if self.gender is not None:
			return gender
		else:
			return 'fail'