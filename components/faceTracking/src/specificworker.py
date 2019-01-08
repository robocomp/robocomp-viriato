import sys, os, traceback, time
import imutils
import time
import datetime
import cv2
from tracker.centroidtracker import CentroidTracker
import numpy as np
from PySide import QtGui, QtCore
from genericworker import *
from PySide.QtCore import QRect, QRectF, Qt, QSize, QSizeF, QPoint, QPointF, QLineF


class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 20
		self.timer.start(self.Period)
		self.model = ''
		self.prototxt = ''
		self.confidence = 0.5
		self.ct = CentroidTracker()
		self.net = None
		self.faces = None


	def setParams(self, params):
		self.model = params['model']
		self.prototxt = params['prototxt']
		self.net = cv2.dnn.readNetFromCaffe(self.prototxt, self.model)
		
		return True

	@QtCore.Slot()
	def compute(self):
        
		color,_, _, _ = self.rgbd_proxy.getData()
		frame = np.fromstring(color, dtype=np.uint8)
		frame = np.reshape(frame,(480, 640, 3))
		frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

		(h, w) = frame.shape[:2]
		blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0,
									 (300, 300), (104.0, 177.0, 123.0))

		# pass the blob through the network and obtain the detections and
		# predictions
		self.net.setInput(blob)
		detections = self.net.forward()
		rects = []

		bounding_boxes = []
		for i in range(0, detections.shape[2]):

			confidence = detections[0, 0, i, 2]
			if confidence >self.confidence:

				box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
				(startX, startY, endX, endY) = box.astype("int")
				cv2.rectangle(frame, (startX, startY), (endX, endY),
							  (0, 255, 100), 2)

				qr = QRect(QPoint(startX, endY), QPoint(endX,startY))
				bounding_boxes.append(qr)

				rects.append(box.astype("int"))

		objects = self.ct.update(rects)

		# loop over the tracked objects

		faces_found = []

		for (objectID, centroid) in objects.items():
			face = TFace()
			tracking = False
			for qrects in bounding_boxes:
				if qrects.contains(QPoint(centroid[0],centroid[1])):
					face.tracking = True
					bbox = Box()
					(bbox.posx, bbox.posy, bbox.width,bbox.height) = qrects.getRect()


					face.boundingbox = bbox
					tracking = True
					break

			if (tracking == False):
				face.tracking = False

			pnt = Point()
			pnt.x = centroid[0]
			pnt.y = centroid [1]

			face.id = objectID
			face.centroid =  pnt

			# draw both the ID of the object and the centroid of the
			# object on the output frame
			text = "ID {}".format(objectID)

			cv2.putText(frame, text, (centroid[0] - 10, centroid[1] - 10),
						cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
			cv2.circle(frame, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)


			faces_found.append(face)

		self.faces = faces_found
		# show the output frame
		cv2.imshow("Frame", frame)
		cv2.waitKey(1) & 0xFF

		return True

	#
	# getFaces
	#
	def getFaces(self):
		return self.faces
