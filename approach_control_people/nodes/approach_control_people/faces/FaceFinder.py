#!/usr/bin/env python

import rospy
import smach
from smach import state
import smach_ros
import cv2
import cv2.cv
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class FaceFinder(smach.state):

	def __init__(self, outcomes=['one_face', 'faces', 'in_progress', 'fail']):

		smach.State.__init__(self, outcomes=['success','sucess2','in_progress','fail'])
		rospy.Subscriber('/image_raw',Image,self.image_callback)
		self.bridge = CvBridge()
		self.face_cascade = cv2.CascadeClassifier('/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml')
		self.faces_found = None
		self.count = 0

	def callback(self,data):
		try:
			image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
		except CvBridgeError as e:
			print(e)

		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		faces = self.face_cascade.detectMultiScale(
			gray,
			scaleFactor=1.1,
			minNeighbors=10, 
			minSize=(100, 100),
			flags=cv2.cv.CV_HAAR_SCALE_IMAGE
		)

		for (x, y, w, h) in faces:
			x1 = x + int(w * .1)
			x2 = x1 + int(w * .8)

			y1 = y + int(h * .2)
			y2 = y1 + int(h * .8)

			roi_gray = gray[y1:y2, x1:x2]
			cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
		
		cv2.imshow('Face Detection', image)
		cv2.waitKey(1)
		self.faces_found = len(faces)

	def execute(self,userdata):
		time.sleep(1)
		if self.faces_found == 1:
			return 'one_face'
		elif self.faces_found > 1:
			return 'faces'
		elif self.faces_found == 0 and self.count < 1:
			return 'in_progress'
			self.count +=1
		elif self.faces_found == 0 and self.count == 1:
			return 'fail'