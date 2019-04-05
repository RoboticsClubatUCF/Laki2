# Character-Alpha-Numeric Classification 

Convolutional Neural Network implemented on tensorflow with keras for distinguishing 
alpha-numeric characters in an image. Overall goal of project is to be able to
distinguish all decimal numbers and capital letters of the English alphabet.

This script comes with pretrained models for distinguishing 0's, 1's, and 2's.

See the various script files for further documentation and usage. 

Implemented on python 3.6/3.7

## Getting Started

Use the provided testImageGen.py script within this Laki2 repo for generating 
testing and training images. 

You can also download a kaggle dataset of the AUVSI competition here: 
* [Kaggle Dataset](https://www.kaggle.com/gndctrl2mjrtm/auvsi-suas-dataset)

### Prerequisites

FILE STRUCTURE FOR TRAINING AND TESTING DATA:
Directory for the testing and training data should be formatted as the following:

NOTE: Your training data image files should be formatted as <imgDescr>.<imgNumber>.png

```

- data
-- testing_data
---- 0.png
---- 1.png
---- 2.png
... and so one

-- training_data
---- 0.0.png
---- 0.1.png
... 
---- 1.0.png
---- 1.1.png
...
---- 2.0.png
---- 2.0.png

And so on...

```


PYTHON LIBRARY DEPENDENCIES FOR USING PROJECT: 
cv2
os
numpy
tqdm
random
tensorflow
matplotlib
time
pickle

Use pip3 for installing libraries above in python3. 

NOTE: You may have to build OpenCV from source.

NOTE: This model is trained on the tensorflow CPU version. 

```
pip3 install opencv-python os numpy tqdm random tensorflow matplotlib time pickle
```

### Deployment

To create the training model, main.py you should call create_training_model(): 

```
create_training_model("model_name")
```

NOTE: Make sure you have the correct numpy and pickle data files for your trained data. 

To test trained model, in main.py you should remove the call to create_training_model()
and call test_trained_model("model_name")

```
test_trained_model("model_name")
```

## Running the tests

Run test_trained_model("model_name") to test out the model.

The matplotlib example image below will be displayed, using twelve randomly selected
images from the testing directory to make predictions on. 

![Test Data Displayed](example_images/Screen_Shot_2019-04-01.png?raw=true "Test Data Displayed")

## Built With

* [Tensorflow w/ Keras](https://www.tensorflow.org/api_docs) - API used for training the model.
* [OpenCV](https://opencv.org/) - Image processing. 

## Authors

* **Peter Dorsaneo** - *Initial work* - [GitHub](https://github.com/peterdors)

## License

This project is licensed under the MIT License.

## Acknowledgments

* Sentdex's CNN tutorials.

