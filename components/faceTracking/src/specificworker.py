import sys, os, traceback, time
import imutils
import time
import datetime
import cv2
from PySide.QtGui import QImage, QPixmap

from tracker.centroidtracker import CentroidTracker
import numpy as np
from PySide import QtGui, QtCore
from genericworker import *
from PySide.QtCore import QRect, QRectF, Qt, QSize, QSizeF, QPoint, QPointF, QLineF
import pickle
# import extract_embeddings as ExtractEm


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 20
        self.timer.start(self.Period)
        self.model = ''
        self.prototxt = ''
        self.embedding_model = ''
        self.confidence = 0.5
        self.ct = CentroidTracker()
        self.net = None
        self.embedder = None
        self.recognizer = None
        self.label_encoder = None
        self.writting_name = False

        self.faces = None # lo que se envia a humanPose

        self.ui.name_lineEdit.textEdited.connect(self.name_changed)
        self.ui.name_lineEdit.editingFinished.connect(self.name_finished)

    def setParams(self, params):
        print("[INFO] loading face detector...")
        self.model = params['model']
        self.prototxt = params['prototxt']
        self.net = cv2.dnn.readNetFromCaffe(self.prototxt, self.model)

        print("[INFO] loading face recognizer...")
        self.embedding_model = params['embedding_model']
        self.embedder = cv2.dnn.readNetFromTorch(self.embedding_model)

        self.recognizer = pickle.loads(open(params['recognizer']).read())
        self.label_encoder = pickle.loads(open(params['le']).read())

        self.show()

        return True


    def name_changed(self,text):
        self.writting_name = True

    def name_finished(self):
        self.writting_name = False

    @QtCore.Slot()
    def compute(self):

        if self.writting_name is False:

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

                    # # extract the face ROI
                    # face = frame[startY:endY, startX:endX]
                    # (fH, fW) = face.shape[:2]
                    #
                    # # ensure the face width and height are sufficiently large
                    # if fW < 20 or fH < 20:
                    #     continue
                    #
                    # # construct a blob for the face ROI, then pass the blob
                    # # through our face embedding model to obtain the 128-d
                    # # quantification of the face
                    # faceBlob = cv2.dnn.blobFromImage(face, 1.0 / 255,
                    #                                  (96, 96), (0, 0, 0), swapRB=True, crop=False)
                    # self.embedder.setInput(faceBlob)
                    # vec = self.embedder.forward()
                    #
                    # # perform classification to recognize the face
                    # preds = self.recognizer.predict_proba(vec)[0]
                    #
                    # j = np.argmax(preds)
                    #
                    # proba = preds[j]
                    # name = self.label_encoder.classes_[j]
                    #
                    # # draw the bounding box of the face along with the
                    # # associated probability
                    # text = "{}: {:.2f}%".format(name, proba * 100)
                    # y = startY - 10 if startY - 10 > 10 else startY + 10
                    # cv2.rectangle(frame, (startX, startY), (endX, endY),
                    #               (255, 10, 150), 2)
                    #
                    # cv2.putText(frame, text, (startX, y),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 10, 150), 2)


                    qr = QRect(QPoint(startX, endY), QPoint(endX,startY))
                    bounding_boxes.append(qr)

                    rects.append(box.astype("int"))

            objects = self.ct.update(rects)


            # loop over the tracked objects

            faces_found = []

            for (objectID, centroid) in objects.items():
                faceT = TFace()
                tracking = False
                for qrects in bounding_boxes:

                    if qrects.contains(QPoint(centroid[0],centroid[1])):
                        faceT.tracking = True
                        bbox = Box()
                        (bbox.posx, bbox.posy, bbox.width,bbox.height) = qrects.getRect()


                        faceT.boundingbox = bbox
                        tracking = True

                        (endX, startY , endX, startY) = qrects.getCoords()

                        # extract the face ROI
                        face = frame[startY:endY, startX:endX]
                        (fH, fW) = face.shape[:2]

                        # ensure the face width and height are sufficiently large
                        if fW < 20 or fH < 20:
                            continue

                        faceBlob = cv2.dnn.blobFromImage(face, 1.0 / 255,
                                                         (96, 96), (0, 0, 0), swapRB=True, crop=False)
                        self.embedder.setInput(faceBlob)
                        vec = self.embedder.forward()

                        # perform classification to recognize the face
                        preds = self.recognizer.predict_proba(vec)[0]

                        j = np.argmax(preds)

                        proba = preds[j]
                        name = self.label_encoder.classes_[j]

                        # draw the bounding box of the face along with the
                        # associated probability
                        text = "{}: {:.2f}%".format(name, proba * 100)
                        y = startY - 10 if startY - 10 > 10 else startY + 10
                        cv2.rectangle(frame, (startX, startY), (endX, endY),
                                      (255, 10, 150), 2)

                        cv2.putText(frame, text, (startX, y),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 10, 150), 2)

                        faceT.name = name
                        faceT.confidence = proba*100

                        break

                if (tracking == False):
                    faceT.tracking = False

                pnt = Point()
                pnt.x = centroid[0]
                pnt.y = centroid [1]

                faceT.id = objectID
                faceT.centroid =  pnt




                # draw both the ID of the object and the centroid of the
                # object on the output frame
                text = "ID {}".format(objectID)

                cv2.putText(frame, text, (centroid[0] - 10, centroid[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 10, 150), 2)
                cv2.circle(frame, (centroid[0], centroid[1]), 4, (255, 10, 150), -1)


                faces_found.append(faceT)

            self.faces = faces_found


            height, width, channel = frame.shape
            bytesPerLine = 3 * width
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            qImg = QImage(frame.data, width, height, bytesPerLine, QImage.Format_RGB888)

            self.ui.image_label.setPixmap(QPixmap.fromImage(qImg))



            return True

    #
    # getFaces
    #
    def getFaces(self):
        return self.faces
