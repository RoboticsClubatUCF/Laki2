import numpy as np
import cv2
 
im = cv2.imread('test.png')
imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
ret,thresh = cv2.threshold(imgray,127,255,0)
im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

# Set to take 1 for now, later will have to determine which contour is associated 
#   with the shape
cnt = contours[1]
cv2.drawContours(im, [cnt], 0, (255,255,255), 3)
cv2.imwrite('contour_output.jpg',im)