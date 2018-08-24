#file is not function. use as a starting point

import numpy as np
import collections
import cv2
import time
import sys

print('loading video source...')
try:
    #uses built-in webcam for capture device
    cap = cv2.VideoCapture(0)

    #gives capture device time to warmup
    time.sleep(2)

    if cap is None or cap == None:
        raise IOError
except IOError:
    sys.exit('video load failure')

####################
###Color Tracking###
####################

print('executing color tracking code...')

while(True):
    #capture each frame of the video
    ret, frame = cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #range of colors (in HSV) that we want to detect, lower means darker
    lowerBlue = (206,100,71)
    upperBlue = (202, 66, 41)

    #initializes the list of points with a buffer of 64 (can be implemented to
    #be user-specified)
    pts = collections.deque(maxlen=64)

    #blurs the frame
    blurred = cv2.GaussianBlur(frame, (11,11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    #construct a mask for the color, then perform operations to remove any
    #blobs of like color
    mask = cv2.inRange(hsv, lowerBlue, upperBlue)
    mask = cv2.erode(mask, None, iterations = 2)
    mask = cv2.dilate(mask, None, iterations = 2)

    #display the frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

print('exiting color tracking code...')

cap.release()
cv2.destroyAllWindows()
