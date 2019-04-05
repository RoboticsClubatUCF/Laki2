# Peter Dorsaneo
# Loads and tests the trained CNN model against test data. 

import cv2, os, numpy as np
from random import shuffle
from tqdm import tqdm
import tensorflow as tf
import matplotlib.pyplot as plt

from create_training_data import CATEGORIES
from create_training_model import create_training_model, MODEL_DIR
from create_testing_data import prepare

# Constants.
# 
# Place your test images in this directory, with the filename structure of 
# 0.png, 1.png, etc. 
# These should be images not used in the training data, but for now we can use those. 
TEST_DIR = 'data/testing_data/'
TEST_DATA_NPY = 'testing_data.npy'

# Run a test viewing of the trained model by accessing it under the model_name 
# parameter.
def test_trained_model(model_name="CNN.model"):

	print("[INFO] Running test on trained model {}".format(model_name))
	model_dir = os.path.join(MODEL_DIR, model_name)
	model = tf.keras.models.load_model(model_dir)

	print('\n\n')

	# Create the matplot figure. 
	fig=plt.figure()
	num = 0

	test_dir = os.listdir(TEST_DIR)

	# Randomize the test directory listing. 
	shuffle(test_dir)

	# MATPLOTLIB PRINTOUT OF RESULTS
	# ====================================================
	for img in test_dir[:12]:
		img_path = os.path.join(TEST_DIR, img)
		prediction = model.predict([prepare(img_path)])

		y = fig.add_subplot(3, 4, num+1)
		label = CATEGORIES[np.argmax(prediction)]

		y.imshow(cv2.imread(img_path), cmap='gray')

		plt.title(label)
		y.axes.get_xaxis().set_visible(False)
		y.axes.get_yaxis().set_visible(False)

		num += 1

		# print(img)
		print(prediction)
		print("\n\n")
		# print(CATEGORIES[np.argmax(prediction)])

	plt.show()
	# ====================================================


	# COMMAND LINE PRINTOUT OF RESULTS
	# ====================================================

	# for img in test_dir[:12]:
	# 	img_path = os.path.join(TEST_DIR, img)

	# 	prediction = model.predict([prepare(img_path)])

	# 	print(img)
	# 	print(prediction)  # will be a list in a list.
	# 	print(CATEGORIES[np.argmax(prediction)])
	# 	print("\n\n")

	# ====================================================

	# For clarity on terminal.
	print('\n\n')

