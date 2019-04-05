# Peter Dorsaneo

import cv2, os, numpy as np
from tqdm import tqdm
from random import shuffle

# Constants.
TRAIN_DIR = 'data/training_data/'
IMG_SIZE = 50
TRAIN_DATA_NPY = 'training_data.npy'
CATEGORIES = ['A', 'B', 'C']

def gray_scale_image(image):
	return cv.cvtColor(image, cv.COLOR_BGR2GRAY)

# For labeling the images 0 thru 9 and A thru Z using a one_hot array. Spicy!
def label_img(img):
	# You will see use of the following python function calls:
	# chr() - takes type int, converts to character value.
	# ord() - takes type char (or string of length 1), converts to int value.

	char_label = img.split('.', 1)[0]

	one_hot = []

	# Build one_hot array with all 0's.
	for i in range(0, len(CATEGORIES)):
		one_hot.append(0)

	one_hot[CATEGORIES.index(char_label)] = 1

	return one_hot

'''
NOTE: File structure should be structed as the follow example outline for this
		method to work:

- TRAIN_DIR 	<- folder directory holding all the training data
------- 0.0.png <- an image with a zero in it. 
...
------- 9.0.png <- an image with a nine in it. 
... 
------- A.0.png <- an image with an A in it. 
...
------- Z.0.png <- an image with a Z in it. 
'''
def create_training_data():
	training_data = []
	print("[INFO] Building training data.")

	for i in tqdm(os.listdir(TRAIN_DIR)):
		try:
			img_path = os.path.join(TRAIN_DIR, i)
			img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
			img = gray_scale_image(img)
			new_img = cv2.resize(img, (IMG_SIZE, IMG_SIZE))
			label = i.split('.', 1)[0]
			training_data.append([new_img, CATEGORIES.index(label)])
		except Exception as e:
			pass


	# Randomize the list of training data rather than have it all in order.
	shuffle(training_data)

	# Save and store trained data as numpy array for later accessing. 
	np.save(TRAIN_DATA_NPY, training_data)

	return training_data



