from sklearn.cluster import DBSCAN as dbscan
import numpy as np
import cv2 as cv
import time
import glob

MIN_PIXEL_WIDTH = 28
MAX_PIXEL_WIDTH = 520
MAX_BACKGROUND_PERCENT = 0.75
WHITE = (255, 255, 255)

def runCluster(image, eps):
	keypoints = getKeypoints(image)
	clusters = dbscan(eps=eps, min_samples=7).fit(keypoints)
	n_clusters = len(set(clusters.labels_))
	return getBounds(clusters, keypoints, 5)

def getKeypoints(image):
	orb = cv.FastFeatureDetector_create()
	keypoints = orb.detect(image, None)
	points=[keys.pt for keys in keypoints]
	return points

def getBounds(clusters, keypoints, border):
	groupedPoints = {}
	bounds = []
	for i, group in enumerate(clusters.labels_):
		if group == -1:
			continue
		if group not in groupedPoints:
			groupedPoints[group] = []
		groupedPoints[group].append(keypoints[i])
	for group in groupedPoints:
		data = np.array(groupedPoints[group]).astype(int)
		x = data[:, 0]
		y = data[:, 1]
		bounds.append([(x.max()+border, y.max()+border), (x.min()-border, y.min()-border)])
	return bounds

def writeFrame(image, bounds):
	for i in bounds:
		cv.rectangle(image, i[0], i[1], (0,0,255), 3)
	cv.imwrite("./rect/image.jpg", image)

def pullImages(image, bounds):
	images=[]
	for boundNum, i in enumerate(bounds):
		img = {}
		width = i[0][0]-i[1][0]
		height = i[0][1]-i[1][1]
		if width>MIN_PIXEL_WIDTH and width<MAX_PIXEL_WIDTH and height<MAX_PIXEL_WIDTH and height>MIN_PIXEL_WIDTH:
			img["centerx"] = i[0][0]+(width/2)
			img["centery"] = i[0][1]+(height/2)
			img["image"] = image[i[1][1]:i[0][1], i[1][0]:i[0][0], :]
			images.append(img)
	return images

def segmentOut():
	input = glob.glob('input/*.jpg')
	for i in range(len(input)):
		name=input[i]
		title = name.split('/')[1].split('.')[0]
		image = cv.imread(name, 1)
		bounds = runCluster(image, 150)
		images = pullImages(image, bounds)
		for i, img in enumerate(images):
			filename = "./output/{0}_{1}-{2}.jpg"
			cv.imwrite(filename.format(title, img["centerx"], img["centery"]), img["image"])

def drawOut():
	input = glob.glob('input/*.jpg')
	for i in range(len(input)):
		name=input[i]
		title = name.split('/')[1].split('.')[0]
		image = cv.imread(name, 1)
		bounds = runCluster(image, 150)
		writeFrame(image, bounds)

#segmentOut()
drawOut()
