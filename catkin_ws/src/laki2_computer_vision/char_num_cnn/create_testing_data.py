# Peter Dorsaneo
# Creates and test the trained model. 
# 
# NOTE: This script is where we will call our training function and test the model.

import cv2, os, numpy as np
from random import shuffle
from tqdm import tqdm
import tensorflow as tf
import matplotlib.pyplot as plt

from create_training_data import CATEGORIES, IMG_SIZE
from create_training_model import create_training_model

# Constants.
# 
# Place your test images in this directory, with the filename structure of 
# 0.png, 1.png, etc. 
# These should be images not used in the training data, but for now we can use those. 
TEST_DIR = 'data/testing_data/'
TEST_DATA_NPY = 'testing_data.npy'

def main():
	# model = create_training_model()

	test_trained_model()


# Create and save a numpy array of the test data for accessing and testing on 
# our neural net. 
def create_testing_data():
	testing_data = []
	for i in tqdm(os.listdir(TEST_DIR)):
		try:
			img_num = i.split('.', 1)[0]
			img = cv2.imread(os.path.join(TEST_DIR, i))
			img = cv2.resize(img, (IMG_SIZE, IMG_SIZE))
			testing_data.append([np.array(img), img_num])
		except Exception as e:
			pass

	shuffle(testing_data)
	np.save(TEST_DATA_NPY, testing_data)

	return testing_data

def prepare(filepath):
	try:
		img_array = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
		new_array = cv2.resize(img_array, (IMG_SIZE, IMG_SIZE))
	except Exception as e:
		pass
	
	return new_array.reshape(-1, IMG_SIZE, IMG_SIZE, 1)




