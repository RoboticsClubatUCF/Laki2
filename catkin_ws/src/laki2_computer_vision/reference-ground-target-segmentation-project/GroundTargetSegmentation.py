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
	#orb = cv.ORB_create()
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

def plotFrame(image, bounds):
	for i in bounds:
		cv.rectangle(image, i[0], i[1], (0,0,255), 3)
	cv.imshow("window", image)

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

def maskBackground(image, colorRange):
	h, w = image["image"].shape[:2]
	image["mask"] = np.full((h+2, w+2), 0, np.uint8)
	#hsv = cv.cvtColor(image["image"], cv.COLOR_BGR2HSV)
	cv.floodFill(image["image"], image["mask"], (0, 0), WHITE, loDiff=(colorRange, colorRange, colorRange), upDiff=(colorRange, colorRange, colorRange), flags=4 | ( 255 << 8 ))
	return image

def getLetter(image, thresh):
	pixelSet = np.float32(image["image"].reshape(-1, 3))
	termCrit = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 10, 1.0)
	nada, labels, colors = cv.kmeans(pixelSet, 3, None, termCrit, 40, cv.KMEANS_RANDOM_CENTERS)
	nada, counts = np.unique(labels, return_counts=True)
	bgIndex = np.where(colors==WHITE)[0]
	counts = np.delete(counts, bgIndex)
	colors = np.delete(colors, bgIndex)
	colorIndexSorted = np.argsort(counts)
	print(colors)
	print("")
	image["shapeColor"] = colors[colorIndexSorted[0]]
	image["letterColor"] = colors[colorIndexSorted[1]]
	image["letter"] = cv.inRange(image["image"], image["letterColor"] - (thresh, thresh, thresh), image["letterColor"] + (thresh, thresh, thresh))
	return image

def videoIn():
	print('loading video source...')
	try:
    		cap = cv.VideoCapture(0)
    		time.sleep(2)
    		if cap is None or cap == None:
        		raise IOError
	except IOError:
    		sys.exit('video load failure')

	while(True):
		ret, frame = cap.read()
		bounds = runCluster(frame, 10)
		plotFrame(frame, bounds)
		if cv.waitKey(1) & 0xFF == ord('q'):
			break

def fileIn():
	input = glob.glob('input/*.jpg')
	for i in range(len(input)):
		name=input[i]
		title = name.split('/')[1].split('.')[0]
		image = cv.imread(name, 1)
		bounds = runCluster(image, 150)
		images = pullImages(image, bounds)
		#writeFrame(image, bounds)
		while False:
			if cv.waitKey(1) & 0xFF == ord('q'):
				break
		for i, img in enumerate(images):
			shape = maskBackground(img, 10)
			average = shape["mask"].mean(axis=0).mean(axis=0)
			if average < (255 * MAX_BACKGROUND_PERCENT):
				shape = getLetter(shape, 10)
				filename = "./output/{0}_{1}-{2}_{3}.jpg"
				cv.imwrite(filename.format(title, shape["centerx"], shape["centery"], "shape"), shape["mask"])
				cv.imwrite(filename.format(title, shape["centerx"], shape["centery"], "image"), shape["image"])
				cv.imwrite(filename.format(title, shape["centerx"], shape["centery"], "letter"), shape["letter"])



fileIn()
#videoIn()



'''
def writeCrops(image, srcName, bounds):
	#filename = "output/{0}_{1}-{2}.png"
	filename = "/Users/darien/Desktop/output/{0}_{1}-{2}.png"
	for i in bounds:
		width = i[0][0]-i[1][0]
		height = i[0][1]-i[1][1]
		centerx = i[0][0]+(width/2)
		centery = i[0][1]+(height/2)
		if width>MIN_PIXEL_WIDTH and width<MAX_PIXEL_WIDTH and height<MAX_PIXEL_WIDTH and height>MIN_PIXEL_WIDTH:
			cv.imwrite(filename.format(srcName, centerx, centery), image[i[1][1]:i[0][1], i[1][0]:i[0][0], :])


'''
