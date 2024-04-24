import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

CAMERA_NUMBER = 0    

#create two named windows:
cv.namedWindow('Input',cv.WINDOW_NORMAL)
cv.namedWindow('Output', cv.WINDOW_NORMAL)

cap = cv.VideoCapture(CAMERA_NUMBER)

cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

#process frames while the camera is open
while(cap.isOpened()  ):

   #read a frame and display it
   ret,frame = cap.read()
   imagePair = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
   imagePair = cv.flip(imagePair,-1)  #I like the USB cable pointing down... 
   cv.imshow('Input', imagePair)

   height,width = imagePair.shape
   #print("Image size:", width, height)

   #We assume side by side images and do no error checking!
   imageL = imagePair[0:height, 0:width//2]
   imageR = imagePair[0:height, width//2:width]


   #Fix the images using camera calibration for much better results here!

   stereo = cv.StereoBM_create()

   #settings could be better I'm sure...
   stereo.setMinDisparity(4)
   stereo.setNumDisparities(128)
   stereo.setBlockSize(21)
   stereo.setSpeckleRange(16)
   stereo.setSpeckleWindowSize(25)

   disparity = stereo.compute(imageL,imageR)
   #print("disparity.dtype", disparity.dtype)
   output = cv.convertScaleAbs(disparity)
   #print("output.dtype", output.dtype)
   cv.imshow('Output', output)
   print("distance: 12")
   
   #press q on the keyboard to exit loop
   if cv.waitKey(20) & 0xFF == ord('q'):
      break;
