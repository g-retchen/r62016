#def dieCacheRead():
# dieCache Read
# based on example from learnopencv.compile

# Standard imports
import cv2
import numpy as np;
import io
import time
import picamera

with picamera.PiCamera() as camera:
     camera.resolution = (1024 , 768);# not used for stream
     camera.start_preview ();
     #camera warm up time
     time.sleep(2);
     camera.capture('dieImage.jpg', resize=(320, 240));
#    with picamera.array.PiRGBArray(camera) as stream:
#    camera.capture(stream, format='bgr')
#    dieImage = stream.array


# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()
 
# # Change thresholds
params.minThreshold = 70;
params.maxThreshold = 250;
 
# #Filter by Area.
params.filterByArea = True
params.minArea = 50
 
# Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.4
 
# Create a detector with the parameters
ver = (cv2.__version__).split('.')
if int(ver[0]) < 3 :
     detector = cv2.SimpleBlobDetector(params)
else: 
     detector = cv2.SimpleBlobDetector_create(params)

# Capture Image
#cap = cv2.VideoCapture(0)
#while(True):
#    ret, frame = cap.read()
#    gray = cv.2 cvtColor(frame,cv2.cCOLOR_BGR@GRAY)
#    cv2.imshow('frame' , gray)
#    if cv2.waitKey(1)&& 0xFF ==ord('q'):
#         break

#cap.release()
#cv2.destroyAllWindows()

     
# Read image

#dieImage = cv2.imread(dieImage, cv2.IMREAD_GRAYSCALE)
 

# Detect blobs.
keypoints = detector.detect(image)
 
# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
im_with_keypoints = cv2.drawKeypoints(dieImage, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
 
# Show keypoints
cv2.imshow("Keypoints", im_with_keypoints)
cv2.waitKey(0)

# Assign count of keypoints to variable for output
dieValue = len(keypoints); 

print ('The die value is ');
print (dieValue);
