#!/usr/bin/python

# Standard imports
import cv2
import numpy as np;

from cv2 import rawcapture
import iostream
from time import sleep
from picamera import PiCamera
from cv2 import rawcapture
import common

# init camera
cam = cv2.camera.capture(0)
cam.preview_start()
sleep(1)
img=cam.capture(0)
cam.preview_stop()
destroyWindow(die)

# take picture
imwrite(img "diecahe.jpg")

# Read image
img = cv2.imread('20161001_120347.jpg', cv2.IMREAD_GRAYSCALE)




# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

# Change thresholds
params.minThreshold = 70
params.maxThreshold = 250

# Filter by Area.
params.filterByArea = True
params.minArea = 50

# Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.4 

# Filter by Convexity
#params.filterByConvexity = True
#params.minConvexity = 0.87
    
# Filter by Inertia
#params.filterByInertia = True
#params.minInertiaRatio = 0.01


# Create a detector with the parameters
ver = (cv2.__version__).split('.')
if int(ver[0]) < 3 :
        detector = cv2.SimpleBlobDetector(params)
else: 
        detector = cv2.SimpleBlobDetector_create(params)

pips=0
count=5
# Detect blobs.
while (pips==0 or pips>6):
        count=count-1
        keypoints = detector.detect(img)
        pips=len(keypoints)
        if (count==0)
            print("Pips is out of Range in 5 tries")
            exit

# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
# the size of the circle corresponds to the size of blob
img_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# Show blobs
cv2.imshow("Keypoints", img_with_keypoints)
cv2.waitKey(5)
dieValue = len(keypoints);
print (dieValue);
