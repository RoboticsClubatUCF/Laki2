import cv2
import numpy as np

# Load in image using imread
image = cv2.imread('test.png')

# Preprocess image. Apply a grayscale, gaussianblur and threshold the image
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray,(5,5),0)
_, thresh = cv2.threshold(gray, 240, 255, cv2.THRESH_OTSU)

# Find contours on the image
contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.ADAPTIVE_THRESH_GAUSSIAN_C)

font = cv2.FONT_HERSHEY_SIMPLEX


# Loop through all the contours
for cnt in contours:
    approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True) # Approximates a contour shape to another shape w/ less vertices.
    img = cv2.drawContours(image, [approx], 0, (0,0,255), 4)
    print(len(approx))

    if (len(approx)) == 3:
        cv2.putText(image,'triangle',(30,50), font, 1,(255,255,255),2,cv2.LINE_AA)
    elif (len(approx)) == 4:
        cv2.putText(image,'square',(30,50), font, 1,(255,255,255),2,cv2.LINE_AA)
    elif (len(approx)) == 5:
        cv2.putText(image,'pentagon',(30,50), font, 1,(255,255,255),2,cv2.LINE_AA)
    elif (len(approx)) == 6:
        cv2.putText(image,'hexagon',(30,50), font, 1,(255,255,255),2,cv2.LINE_AA)
    elif (len(approx)) == 7:
        cv2.putText(image,'heptagon',(30,50), font, 1,(255,255,255),2,cv2.LINE_AA)
    elif (len(approx)) == 8:
        cv2.putText(image,'octagon',(30,50), font, 1,(255,255,255),2,cv2.LINE_AA)


cv2.imshow("result", image)
cv2.waitKey(0)
cv2.destroyAllWindows()