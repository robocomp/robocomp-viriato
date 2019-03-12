# USAGE
# python extract_embeddings.py --dataset dataset --embeddings output/embeddings.pickle \
#	--detector face_detection_model --embedding-model openface_nn4.small2.v1.t7

# import the necessary packages
from imutils import paths
import numpy as np
import argparse
import imutils
import pickle
import cv2
import os
import align_faces as ALF

# construct the argument parser and parse the arguments
def extract_embeddings():

	conf = 0.5

	dataset = "/home/robocomp/robocomp/components/robocomp-viriato/components/faceTracking/files/dataset"

	model = "/home/robocomp/robocomp/components/robocomp-viriato/components/faceTracking/files/res10_300x300_ssd_iter_140000.caffemodel"
	prototxt = "/home/robocomp/robocomp/components/robocomp-viriato/components/faceTracking/files/deploy.prototxt"
	detector = cv2.dnn.readNetFromCaffe(prototxt, model)

	print("[INFO] quantifying faces...")
	imagePaths = list(paths.list_images(dataset))

	embedding_model = "/home/robocomp/robocomp/components/robocomp-viriato/components/faceTracking/files/openface_nn4.small2.v1.t7"
	embedder = cv2.dnn.readNetFromTorch(embedding_model)

	knownEmbeddings = []
	knownNames = []

	# initialize the total number of faces processed
	total = 0
	NoneType = type(None)

	# loop over the image paths
	for (i, imagePath) in enumerate(imagePaths):
		# extract the person name from the image path

		name = imagePath.split(os.path.sep)[-2]

		image_orig = cv2.imread(imagePath)

		if(type(image_orig) == NoneType):
			continue
		image = imutils.resize(image_orig, width=600)
		(h, w) = image_orig.shape[:2]

		################# aliging image ######################
		# if(type(image_orig) == NoneType):
		# 	img_original_good = False
		# else:
		# 	img_original_good = True
		#
		# if (img_original_good == False): continue
		#
		#
		# image_orig = imutils.resize(image_orig, width=800)
		# (h, w) = image_orig.shape[:2]
		#print("[INFO] aligning image {}/{}".format(i + 1, len(imagePaths)))
		# image = ALF.align_faces(image_orig,h,w, detector)
		#
		# if (type(image) == NoneType):
		# 	img_alig_good = False
		# else:
		# 	img_alig_good = True
		#
		# if ((img_alig_good == False) and (img_original_good == True)):
		# 	print("[INFO] Aligned image not valid. Using original" )
		# 	image = image_orig
		#
		# elif(img_alig_good == False) :
		# 	print("[INFO] Can't process image {}/{}".format(i + 1, len(imagePaths)))
		# 	continue
		#########################################################
		print("[INFO] processing image {}/{}".format(i + 1, len(imagePaths)))

		# construct a blob from the image
		imageBlob = cv2.dnn.blobFromImage(
			cv2.resize(image, (300, 300)), 1.0, (300, 300),
			(104.0, 177.0, 123.0), swapRB=False, crop=False)


		detector.setInput(imageBlob)
		detections = detector.forward()

		# ensure at least one face was found
		if len(detections) > 0:
			# we're making the assumption that each image has only ONE
			# face, so find the bounding box with the largest probability
			i = np.argmax(detections[0, 0, :, 2])
			confidence = detections[0, 0, i, 2]

			if confidence > conf:

				box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
				(startX, startY, endX, endY) = box.astype("int")

				face = image[startY:endY, startX:endX]
				(fH, fW) = face.shape[:2]

				if fW < 20 or fH < 20:
					continue

				faceBlob = cv2.dnn.blobFromImage(face, 1.0 / 255,
					(96, 96), (0, 0, 0), swapRB=True, crop=False)
				embedder.setInput(faceBlob)
				vec = embedder.forward()

				knownNames.append(name)
				knownEmbeddings.append(vec.flatten())
				total += 1

	# dump the facial embeddings + names to disk
	print("[INFO] serializing {} encodings...".format(total))
	data = {"embeddings": knownEmbeddings, "names": knownNames}
	f = open("/home/robocomp/robocomp/components/robocomp-viriato/components/faceTracking/files/output/embeddings.pickle", "wb")
	f.write(pickle.dumps(data))
	f.close()
