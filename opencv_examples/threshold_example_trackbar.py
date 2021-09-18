import cv2
import numpy as np
from matplotlib import pyplot as plt


video_capture_device = cv2.VideoCapture(1)
detector = cv2.SimpleBlobDetector_create() #detector

trackbar_value = 127

def updateValue(new_value):
    # Make sure to write the new value into the global variable
    global trackbar_value
    trackbar_value = new_value
    
cv2.namedWindow("Output")
cv2.createTrackbar("Threshhold", "Output", trackbar_value, 255, updateValue)

while True:
    
    Ret, frame = video_capture_device.read() #capture the video
 
    img = cv2.resize(frame, (0,0), fx=1, fy=1) #size
    img = cv2.bitwise_not(img) #invert
    keypoints = detector.detect(img) #detect blops
    ret, th1 = cv2.threshold(img, trackbar_value ,255,cv2.THRESH_BINARY_INV)
    
    cv2.imshow("Output", th1)

    if cv2.waitKey(1) & 0xFF == ord('q'):
       break

video_capture_device.release()

cv2.destroyAllWindows()