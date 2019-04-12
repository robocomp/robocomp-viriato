# USAGE
# python align_faces.py --shape-predictor shape_predictor_68_face_landmarks.dat --image /home/robocomp/robocomp/components/robocomp-viriato/components/faceTracking/files/dataset/araceli/frame19.jpg

# import the necessary packages
from imutils.face_utils import FaceAligner
import imutils
import dlib
import cv2
import numpy as np

def align_faces(image, h, w, net):
    shape_predictor = "/home/robocomp/robocomp/components/robocomp-viriato/components/faceTracking/files/shape_predictor_5_face_landmarks.dat" #faster
    # shape_predictor = "/home/robocomp/robocomp/components/robocomp-viriato/components/faceTracking/files/shape_predictor_68_face_landmarks.dat"

    predictor = dlib.shape_predictor(shape_predictor)

    fa = FaceAligner(predictor, desiredFaceWidth=300, desiredFaceHeight=400)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 1.0,
        (300, 300), (104.0, 177.0, 123.0))

    net.setInput(blob)
    detections = net.forward()

    for i in range(0, detections.shape[2]):
        #     # predictionextract the confidence (i.e., probability) associated with the
        confidence = detections[0, 0, i, 2]

        if confidence > 0.5:
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")

            rect = dlib.rectangle(left=startX, top=startY, right=endX, bottom=endY)
            print (rect)

            faceAligned = fa.align(image, gray, rect)

            return faceAligned

