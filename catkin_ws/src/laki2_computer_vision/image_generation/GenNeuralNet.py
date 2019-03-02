import testImageGen
import keras
import cv2
import numpy as np
import random
import os
from glob import glob
from keras.preprocessing.image import ImageDataGenerator
from keras.models import Sequential, load_model, save_model
from keras.layers import Conv2D, MaxPooling2D
from keras.layers import Activation, Dropout, Flatten, Dense
from keras import backend as K
from tqdm import tqdm

 

TRAIN_DIR = os.path.dirname(os.path.realpath(__file__)) + "/generatedImages"
MODEL_NAME = "Dab"

testRange = 50
'''
for i in range(0, testRange):
    testImageGen.generateImage(400, 5, 10, str(i))
'''
# dimensions of our images.
img_width, img_height = 400, 400

def label_img(imageFile):
    file = open(TRAIN_DIR + "/" + str(imageFile) + ".txt", "r", encoding='latin-1')
    shape = file.readline()
    shape = shape.split()[1]
    if(shape == "Circle"): return [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    if(shape == "Square"): return [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    if(shape == "Rectangle"): return [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    if(shape == "Semi"): return [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0]
    if(shape == "Quarter"): return [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
    if(shape == "Triangle"): return [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
    if(shape == "Pentagon"): return [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]
    if(shape == "Hexagon"): return [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0]
    if(shape == "Cross"): return [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
    if(shape == "Trapezoid"): return [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
    if(shape == "Heptagon"): return [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0]
    if(shape == "Octogon"): return [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]

def create_train_data():
    training_data = []
    for i in range(0,testRange):
        label = label_img(i)
        img = cv2.imread(TRAIN_DIR + "/" + str(i) + ".png", cv2.IMREAD_COLOR)
        img = cv2.resize(img, (400, 400))
        training_data.append([np.array(img),label])
    random.shuffle(training_data)
    np.save('train_data.npy', training_data)
    return training_data

train = create_train_data()

train_data_dir = 'image_generation/'
validation_data_dir = 'image_generation'
nb_train_samples = 2000
nb_validation_samples = 800
epochs = 50

if K.image_data_format() == 'channels_first':
    input_shape = (3, img_width, img_height)
else:
    input_shape = (img_width, img_height, 3)

model = Sequential()
model.add(Conv2D(64, (3, 3), input_shape=input_shape))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Conv2D(128, (3, 3)))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Conv2D(64, (3, 3)))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Conv2D(64, (3, 3)))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Flatten())
model.add(Dense(64))
model.add(Activation('relu'))
model.add(Dense(24))
model.add(Dense(12, activation='sigmoid'))

model.compile(loss='categorical_crossentropy',
              optimizer='adam',
              metrics=['accuracy'])

'''
if os.path.exists('{}.h5'.format(MODEL_NAME)):
    model = load_model('{}.h5'.format(MODEL_NAME))
    print("Model loaded!")
    print("--------------------------------------")
'''

X = np.array([i[0] for i in train]).reshape(-1, 400, 400,3)
Y = np.array([i[1] for i in train])

model.fit(X, Y, epochs=50)
#print(X)
#Y = model.predict_classes(X,1)
#print(Y)
model.save('{}.h5'.format(MODEL_NAME))
print("Model saved!")
