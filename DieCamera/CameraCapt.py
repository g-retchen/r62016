import numpy
import cv2
from picamera.array import PiRGBArray #Camera stuff
from picamera import PiCamera #Camera Stuff
import time #Used for a brief delay

camera=PiCamera() #Camera set-up
rawCapture=PiRGBArray(camera) #Unprocessed array

time.sleep(2) #Wait a bit
camera.capture(rawCapture,format="bgr") #capture an image of RGB
image=rawCapture.array #Save the array as variable image

cv2.imshow("Image", image) #Show the array
cv2.waitKey(0)

cv2.imwrite("img.jpg",image) #Save to disc
   
