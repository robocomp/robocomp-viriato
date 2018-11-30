import sys, os, traceback, time
import imutils
import time
import cv2
import centroidtracker as CentroidTracker
from imutils.video import VideoStream
import numpy as np
from PySide import QtGui, QtCore
from genericworker import *


class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)
		self.model = ''
		self.prototxt = ''
		self.confidence = 0.5
		self.ct = CentroidTracker()
		self.width = None
		self.height = None
		self.net = None

	def setParams(self, params):
		self.model = params['model']
		self.prototxt = params['prototxt']

		self.net = cv2.dnn.readNetFromCaffe(self.prototxt, self.model)
		return True

	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'


		# initialize the video stream and allow the camera sensor to warmup
		# print("[INFO] starting video stream...")
		# vs = VideoStream(src=0).start()
		# time.sleep(2.0)

		# read the next frame from the video stream and resize it
		# frame = vs.read()
		# frame = imutils.resize(frame, width=400)

		color,_, _, _ = self.rgbd_proxy.getData()
		frame = np.fromstring(color, dtype=np.uint8)
		frame = frame.reshape(480, 640, 3)

		# if the frame dimensions are None, grab them
		if self.width is None or self.height is None:
			(self.width, self.height) = frame.shape[:2]

		# construct a blob from the frame, pass it through the network,
		# obtain our output predictions, and initialize the list of
		# bounding box rectangles
		blob = cv2.dnn.blobFromImage(frame, 1.0, (self.width, self.height),
									 (104.0, 177.0, 123.0))
		self.net.setInput(blob)
		detections = self.net.forward()
		rects = []

		# loop over the detections
		for i in range(0, detections.shape[2]):
			# filter out weak detections by ensuring the predicted
			# probability is greater than a minimum threshold
			if detections[0, 0, i, 2] > self.confidence:
				# compute the (x, y)-coordinates of the bounding box for
				# the object, then update the bounding box rectangles list
				box = detections[0, 0, i, 3:7] * np.array([W, H, W, H])
				rects.append(box.astype("int"))

				# draw a bounding box surrounding the object so we can
				# visualize it
				(startX, startY, endX, endY) = box.astype("int")
				cv2.rectangle(frame, (startX, startY), (endX, endY),
							  (0, 255, 0), 2)

		# update our centroid tracker using the computed set of bounding
		# box rectangles
		objects = self.ct.update(rects)

		# loop over the tracked objects
		for (objectID, centroid) in objects.items():
			# draw both the ID of the object and the centroid of the
			# object on the output frame
			text = "ID {}".format(objectID)
			cv2.putText(frame, text, (centroid[0] - 10, centroid[1] - 10),
						cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
			cv2.circle(frame, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)

		# show the output frame
		cv2.imshow("Frame", frame)
		key = cv2.waitKey(1) & 0xFF

		# if the `q` key was pressed, break from the loop
		if key == ord("q"):
			exit()

		cv2.destroyAllWindows()
		vs.stop()

		return True

