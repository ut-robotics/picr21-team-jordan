import cv2
import imutils
import time



vs = cv2.VideoCapture(4)

cv2.namedWindow("main")
cv2.namedWindow("mask")


def updateValue(new_value):
    global trackbar_value
    trackbar_value = new_value

cv2.createTrackbar('hl', "mask", 0, 255, updateValue)
cv2.createTrackbar('sl', "mask", 0, 255, updateValue)
cv2.createTrackbar('vl', "mask", 0, 255, updateValue)
cv2.createTrackbar('hh', "mask", 0, 255, updateValue)
cv2.createTrackbar('sh', "mask", 0, 255, updateValue)
cv2.createTrackbar('vh', "mask", 0, 255, updateValue)

while True:
    _, frame = vs.read()

    if frame is None:
        break
    hl = cv2.getTrackbarPos('hl', "mask")
    sl = cv2.getTrackbarPos('sl', "mask")
    vl = cv2.getTrackbarPos('vl', "mask")
    hh = cv2.getTrackbarPos('hh', "mask")
    sh = cv2.getTrackbarPos('sh', "mask")
    vh = cv2.getTrackbarPos('vh', "mask")
    greenLower = (hl, sl, vl)
    greenUpper = (hh, sh, vh)

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    width, height = frame.shape[:2]
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # To see the centroid clearly
        if radius > 10:
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 5)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

    cv2.imshow("main", frame)
    cv2.imshow("mask", mask)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vs.release()
cv2.destroyAllWindows()