from __future__ import print_function
from imutils.video import VideoStream
import numpy as np
import numpy
import datetime
import time
import imutils
import time
import cv2
import picamera  
#from picamera.array import PiRGBArray
from picamera import PiCamera
#
print ("[INFO] starting camera...")
webcam = VideoStream(src=0).start()
picam = VideoStream(usePiCamera=True).start()
time.sleep( 2 )
#
camera = PiCamera()
camera_resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640,480)
#
#time.sleep( 0.5 )
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    cv2.imshow("Frame", image)
    key = cv2.waitKey(1) & 0xFF

    rawCapture.truncate(0)
    if key == ord("q"):
        break
    
                        
