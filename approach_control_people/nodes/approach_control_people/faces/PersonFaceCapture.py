#!/usr/bin/env python

import rospy
import smach
import os
from smach import state
import smach_ros
import cv2
import cv2.cv
import imutils
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class PersonFaceCapture(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['success','in_progress','fail'])
		rospy.Subscriber('/image_raw', Image, self.callback)
		self.capture = 0
		#self.faces_db_dir = rospy.get_param('~face_database_path')
		self.face_cascade = cv2.CascadeClassifier('/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml')
		self.tmp_dir = '~/faces/tmp'
		self.image_saved = False
		self.count = 0

	def callback(self,data):

		try:
		    image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
		except CvBridgeError as e:
		    print(e)

		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		roi_gray = None
		utils = Utils()

		faces = utils.detect_faces(gray)

		if self.capture < 25:

		    self.capture_operator = self.capture_operator + 1
		    print str(self.capture_operator)
		    for (x, y, w, h) in faces:
		        x1 = x + int(w * .1)
		        x2 = x1 + int(w * .8)

		        y1 = y + int(h * .2)
		        y2 = y1 + int(h * .8)

		        roi_gray = gray[y1:y2, x1:x2]
		        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

		        if roi_gray is not None:
		            face_numbers = len(os.listdir(self.tmp_dir))
		            file_name = '{0}.png'.format(face_numbers)
		            utils.save_image_opencv(
		                self.tmp_dir,
		                file_name,
		                utils.resize_image_opencv_128x128(roi_gray)
		            )
		else:
			utils.save_image_opencv(self.tmp_dir, 'operator.png', fu.grey_image_mean(self.tmp_dir))
			# utils.save_image_opencv(self.faces_db_dir, 'operator.png', fu.grey_image_mean(self.tmp_dir))
			self.image_saved = True

	def execute(self,userdata):

		time.sleep(5)
		if self.image_saved:
			return 'success'
		elif self.capture < 25:
			return 'in_progress'
		else:
			return 'fail'