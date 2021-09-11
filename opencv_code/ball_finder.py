import time
import cv2
import input_manager
import numpy as np


cap = cv2.VideoCapture(4)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cv2.namedWindow('Original')


path = "/home/jordan_team/picr21-team-jordan/opencv_code/"


def updateValue(new_value):
    global trackbar_valueq
    trackbar_value = new_value

default_values = input_manager.get_default_values(path)
cv2.createTrackbar('hl', 'Original', default_values[0], 255, updateValue)
cv2.createTrackbar('sl', 'Original', default_values[1], 255, updateValue)
cv2.createTrackbar('vl', 'Original', default_values[2], 255, updateValue)
cv2.createTrackbar('hh', 'Original', default_values[3], 255, updateValue)
cv2.createTrackbar('sh', 'Original', default_values[4], 255, updateValue)
cv2.createTrackbar('vh', 'Original', default_values[5], 255, updateValue)
fps = 0

while True:
    start_time = time.time()
    ret, frame = cap.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    hl = cv2.getTrackbarPos('hl', 'Original')
    sl = cv2.getTrackbarPos('sl', 'Original')
    vl = cv2.getTrackbarPos('vl', 'Original')
    hh = cv2.getTrackbarPos('hh', 'Original')
    sh = cv2.getTrackbarPos('sh', 'Original')
    vh = cv2.getTrackbarPos('vh', 'Original')
    lowerLimits = np.array([hl, sl, vl])
    upperLimits = np.array([hh, sh, vh])
    frame = cv2.medianBlur(frame, 7)
    thresh = cv2.inRange(frame, lowerLimits, upperLimits)
    thresh = cv2.bitwise_not(thresh)

    #outimage = cv2.bitwise_and(frame, frame, mask = thresh)
    
    cv2.putText(frame, str(fps), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow('Original', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        input_manager.save_default_values(path, [str(x) for x in [hl, sl, vl, hh, sh, vh]])
        break

    fps = round(1.0 / (time.time() - start_time), 2)

cap.release()
cv2.destroyAllWindows()
