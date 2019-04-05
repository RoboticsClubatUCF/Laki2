import tensorflow as tf
from tensorflow.keras.datasets import cifar10
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Dropout, Activation, Flatten
from tensorflow.keras.layers import Conv2D, MaxPooling2D
from tensorflow.keras.callbacks import TensorBoard
# Import create_training_data method and some constants.
from create_training_data import create_training_data, TRAIN_DATA_NPY, IMG_SIZE
from tqdm import tqdm
import os, time, numpy as np
import pickle


# Constants.
FEAT_PKL = 'X.pickle'
LABEL_PKL = 'y.pickle'
MODEL_DIR = 'models'

# Builds the numpy arrays for the features and labels of the training model.
# Utilizes pickle library for saving and loading binary files.
def build_features_and_labels(training_data):
	X = []
	y = []

	print("[INFO] Building features and labels.")
	
	for features, label in tqdm(training_data):
		X.append(features)
		y.append(label)

	# print(X[0].reshape(-1, IMG_SIZE, IMG_SIZE, 1))

	X = np.array(X).reshape(-1, IMG_SIZE, IMG_SIZE, 1)
	
	pickle_out = open(FEAT_PKL, "wb")
	pickle.dump(X, pickle_out)
	pickle_out.close()

	pickle_out = open(LABEL_PKL, "wb")
	pickle.dump(y, pickle_out)
	pickle_out.close()


# Implements and processes the training epochs for the model.
# Saves a copy of the model, named under the parameter model_name.
def create_training_model(model_name="CNN.model"):

	if os.path.exists(TRAIN_DATA_NPY):
		train_data = np.load(TRAIN_DATA_NPY)
	else:
		train_data = create_training_data()

	build_features_and_labels(train_data)

	pickle_in = open(FEAT_PKL, "rb")
	X = pickle.load(pickle_in)

	pickle_in = open(LABEL_PKL, "rb")
	y = pickle.load(pickle_in)

	# Normalize/scale image pixel data by dividing by the max pixel value
	X = X / 255.0

	# These layers/sizes are for finding the best suited range. 
	# dense_layers = [0, 1, 2]
	# layer_sizes = [32, 64, 128]
	# conv_layers = [1, 2, 3]

	# These layers/sizes worked best with detecting 0's and 1's, and 2's.
	dense_layers = [1]
	layer_sizes = [128]
	conv_layers = [3]

	print("[INFO] Creating training model.")

	for dense_layer in dense_layers:
		for layer_size in layer_sizes:
			for conv_layer in conv_layers:
				NAME = "{}-conv-{}-nodes-{}-dense-{}".format(conv_layer, layer_size, dense_layer, int(time.time()))
				print(NAME)

				model = Sequential()

				model.add(Conv2D(layer_size, (3, 3), input_shape=X.shape[1:]))
				model.add(Activation('relu'))
				model.add(MaxPooling2D(pool_size=(2, 2)))

				for l in range(conv_layer-1):
					model.add(Conv2D(layer_size, (3, 3)))
					model.add(Activation('relu'))
					model.add(MaxPooling2D(pool_size=(2, 2)))

				model.add(Flatten())

				for _ in range(dense_layer):
					model.add(Dense(layer_size))
					model.add(Activation('relu'))

				model.add(Dense(3))
				model.add(Activation('softmax'))

				tensorboard = TensorBoard(log_dir="logs/{}".format(NAME))

				model.compile(loss='sparse_categorical_crossentropy',
								optimizer='adam',
								metrics=['accuracy'],
								)

				model.fit(np.array(X), np.array(y),
							batch_size=32,
							epochs=4,
							validation_split=0.3,
							callbacks=[tensorboard])

	if not os.path.exists(MODEL_DIR):
		os.mkdir(MODEL_DIR)
		
	model.save(os.path.join(MODEL_DIR, model_name))

	return model

