# Built and modified code from: https://www.learnopencv.com/keras-tutorial-fine-tuning-using-pre-trained-models/

# Largely unorganized, but it works as a demo to retrain VGG16 to classify custom
#   classes given training data

from keras.applications import VGG16
from keras import models
from keras import layers
from keras import optimizers
from keras.preprocessing import image as kPPimage
import matplotlib.pyplot as plt

#Load the VGG model
image_size = 224
vgg_conv = VGG16(weights='imagenet', include_top=False, input_shape=(image_size, image_size, 3))

# Freeze the layers except the last 4 layers
for layer in vgg_conv.layers[:-4]:
    layer.trainable = False
 
# Check the trainable status of the individual layers
for layer in vgg_conv.layers:
    print(layer, layer.trainable)
 
# Create the model
model = models.Sequential()
 
# Add the vgg convolutional base model
model.add(vgg_conv)
 
# Add new layers
model.add(layers.Flatten())
model.add(layers.Dense(1024, activation='relu'))
model.add(layers.Dropout(0.5))
# The first input here refers to the number of possible outputs
model.add(layers.Dense(12, activation='softmax'))
 
# Show a summary of the model. Check the number of trainable parameters
model.summary()

train_datagen = kPPimage.ImageDataGenerator(
      rescale=1./255,
      rotation_range=20,
      width_shift_range=0.2,
      height_shift_range=0.2,
      horizontal_flip=True,
      fill_mode='nearest')
 
validation_datagen = kPPimage.ImageDataGenerator(rescale=1./255)
 
# Change the batchsize according to your system RAM
train_batchsize = 15
val_batchsize = 10

# 'train' is just the name of training data
train_generator = train_datagen.flow_from_directory(
        'train',
        target_size=(image_size, image_size),
        batch_size=train_batchsize,
        class_mode='categorical')
 
# 'test' is just the name of training data
validation_generator = validation_datagen.flow_from_directory(
        'test',
        target_size=(image_size, image_size),
        batch_size=val_batchsize,
        class_mode='categorical',
        shuffle=False)

# Compile the model
model.compile(loss='categorical_crossentropy',
              optimizer=optimizers.RMSprop(lr=1e-4),
              metrics=['acc'])

# Train the model
history = model.fit_generator(
      train_generator,
      steps_per_epoch=train_generator.samples/train_generator.batch_size,
      epochs=10,
      validation_data=validation_generator,
      validation_steps=validation_generator.samples/validation_generator.batch_size,
      verbose=1)
 
# Save the model
model.save('small_last4.h5')

acc = history.history['acc']
val_acc = history.history['val_acc']
loss = history.history['loss']
val_loss = history.history['val_loss']
 
epochs = range(len(acc))
 
plt.plot(epochs, acc, 'b', label='Training acc')
plt.plot(epochs, val_acc, 'r', label='Validation acc')
plt.title('Training and validation accuracy')
plt.legend()
 
plt.figure()
 
plt.plot(epochs, loss, 'b', label='Training loss')
plt.plot(epochs, val_loss, 'r', label='Validation loss')
plt.title('Training and validation loss')
plt.legend()
 
plt.show()