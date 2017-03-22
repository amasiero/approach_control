#!/usr/bin/env python

import rospy
import smach
import os
import os.path
from smach import state
import smach_ros
import cv2
import cv2.cv
import imutils
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from approach_control_people.faces import Utils

class PersonFaceCapture(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['success','in_progress','fail'])
		self.capture = 0
		#self.faces_db_dir = rospy.get_param('~face_database_path')
		self.face_cascade = cv2.CascadeClassifier('/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml')
		self.tmp_dir = os.path.abspath('catkin_ws/src/approach_control/database/faces/tmp/') #verificar como pegar o diretorio a partir do home com o comando ~
		self.image_saved = False
		self.bridge = CvBridge()

	def callback(self,data):

		try:
		    image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
		except CvBridgeError as e:
		    print(e)

		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

		roi_gray = None
		utils = Utils.Utils()

		faces = self.face_cascade.detectMultiScale(
			gray,
			scaleFactor = 1.1,
			minNeighbors = 10,
			minSize = (100, 100),
			flags = cv2.cv.CV_HAAR_SCALE_IMAGE
		)

		if self.capture < 25:
		    
		    print str(self.capture)
		    for (x, y, w, h) in faces:
		        x1 = x + int(w * .1)
		        x2 = x1 + int(w * .8)

		        y1 = y + int(h * .2)
		        y2 = y1 + int(h * .8)

		        roi_gray = gray[y1:y2, x1:x2]
		        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

		        cv2.imshow('Cropped Image', roi_gray)

		        if roi_gray is not None:
		            face_numbers = len(os.listdir(self.tmp_dir))
		            file_name = '{0}.png'.format(face_numbers)
		            utils.save_image_opencv(
		                self.tmp_dir,
		                file_name,
		                utils.resize_image_opencv_128x128(roi_gray)
		            )
		            self.capture = self.capture + 1
		else:
			utils.save_image_opencv(os.path.abspath('catkin_ws/src/approach_control/database/faces/'), 'operator.png', utils.grey_image_mean(self.tmp_dir))
			# utils.save_image_opencv(self.faces_db_dir, 'operator.png', utils.grey_image_mean(self.tmp_dir))
			self.image_saved = True

	def execute(self,userdata):
		rospy.Subscriber('/image_raw', Image, self.callback)
		rospy.sleep(5)
		if self.image_saved:
			return 'success'
		elif self.capture < 25:
			return 'in_progress'
		else:
			return 'fail'