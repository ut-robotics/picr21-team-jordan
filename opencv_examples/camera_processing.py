"""
This code contains example of image processing in hsv format. Programm should
detect ball coordinates after proper threshold.
This is unoptimized code, should! be refactored.
Used pyhton 3.7.0
"""

import numpy as np
import cv2
import time
import os

cap = cv2.VideoCapture(4)

def updateValue(new_value):
    global trackbar_value
    trackbar_value = new_value
       
cv2.namedWindow('Original')
cv2.namedWindow('Processed')

# config file
cfg_dir = os.listdir("/home/")
if "trackbar_defaults.txt" in cfg_dir:
    cfg = open("trackbar_defaults.txt")
    c = []
    for line in cfg:
        line = int(line)
        c.append(line)
    cfg.close()
else:
    c = [0,0,0,255,255,255]



cv2.createTrackbar('hl', 'Processed', c[0], 255, updateValue)
cv2.createTrackbar('sl', 'Processed', c[1], 255, updateValue)
cv2.createTrackbar('vl', 'Processed', c[2], 255, updateValue)
cv2.createTrackbar('hh', 'Processed', c[3], 255, updateValue)
cv2.createTrackbar('sh', 'Processed', c[4], 255, updateValue)
cv2.createTrackbar('vh', 'Processed', c[5], 255, updateValue)
# cv2.createTrackbar('blur', 'Original', 1, 10, updateValue)

blobparams = cv2.SimpleBlobDetector_Params()
blobparams.filterByArea = True
blobparams.minArea = 999
blobparams.maxArea = 999999
blobparams.filterByInertia = False
blobparams.filterByConvexity = False
detector = cv2.SimpleBlobDetector_create(blobparams)

#fps
a = []
time1 = time.time()
fps = 0

while True:
    
    #fps
    time2 = time.time()
    vahe = time2 - time1
    if vahe == 0:
        pass
    else:
        fps = round(1.0/vahe, 2)
        
    time1 = time2
    a.append(fps)
    fps1 = int(sum(a)/len(a))
    ##
    
    ret, frame = cap.read()
    
#     hvsimg = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) if i want to use this, replace frame with hvsimg
    

    hl = cv2.getTrackbarPos('hl', 'Processed')
    sl = cv2.getTrackbarPos('sl', 'Processed')
    vl = cv2.getTrackbarPos('vl', 'Processed')
    hh = cv2.getTrackbarPos('hh', 'Processed')
    sh = cv2.getTrackbarPos('sh', 'Processed')
    vh = cv2.getTrackbarPos('vh', 'Processed')
#     blur = cv2.getTrackbarPos('blur', 'Original')
    
    lowerLimits = np.array([hl, sl, vl])
    upperLimits = np.array([hh, sh, vh])
    
    frame = cv2.medianBlur(frame, 7) #blur1
#     frame = cv2.bilateralFilter(frame,20,75,75)
    
    thresh = cv2.inRange(frame, lowerLimits, upperLimits)
    thresh = cv2.bitwise_not(thresh)
    #outimage = cv2.bitwise_and(frame, frame, mask = thresh)
    
    
    keypoints = detector.detect(thresh)
    cv2.putText(frame, str(fps1), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.drawKeypoints(frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    for keypoint in keypoints:
        xy = "  x: " + str(int(keypoint.pt[0])) + " y: " + str(int(keypoint.pt[0]))
        cv2.putText(frame, xy , (int(keypoint.pt[0]), int(keypoint.pt[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    kernel = np.ones((5,5),np.float32)/25
    
#     result = cv2.filter2D(frame,-1,kernel)

    
    cv2.imshow('Original', frame)
    cv2.imshow('Processed', thresh)
    
    ##fps 2
    if len(a) >10:
        a = []

    if cv2.waitKey(1) & 0xFF == ord('q'):
        ##config part 2
        c = [hl, sl, vl, hh, sh, vh]
        cfg = open("trackbar_defaults.txt", "w")
        for i in c:
            i = str(i) + "\n"
            cfg.write(i)
        cfg.close()
        break

print('closing program')
cap.release()
cv2.destroyAllWindows()
