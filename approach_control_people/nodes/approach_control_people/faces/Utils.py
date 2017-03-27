#!/usr/bin/env python

"""
@author: Andrey Masiero
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt

from os import listdir
from os.path import exists
from skimage import io
from PIL import Image

class Utils(object):

	def __init__(self):
		self.face_cascade = cv2.CascadeClassifier('/usr/local/share/OpenCV/haarcascade_frontalface_default.xml')

	def load_grey_image_skimage(self, file):
		return io.imread(file, as_grey=True)

	def load_grey_image_opencv(self, file):
		# param 0 means grey scale image
		return cv2.imread(file, 0)

	def load_image_opencv(self, file):
		return cv2.imread(file)

	def resize_image_pil_128x128(self, file):
		return Image.open(file).resize((128, 128), Image.ANTIALIAS)

	def resize_image_opencv_128x128(self, image):
		return cv2.resize(image, (128, 128))

	def grey_image_mean(self, path):
		return np.mean([self.load_grey_image_skimage(path + file) for file in listdir(path) if '.png' in file], 0)

	def save_grey_image_skimage(self, path, filename, image):
		plt.imsave(path + filename, image, cmap=plt.cm.gray)

	def save_image_opencv(self, path, filename, image):
		cv2.imwrite(path + filename, image)

	def detect_faces(self, image):
		faces = self.face_cascade.detectMultiScale(
			image,
			scaleFactor = 1.1,
			minNeighbors = 10,
			minSize = (100, 100),
			flags = cv2.cv.CV_HAAR_SCALE_IMAGE
		)
		return faces