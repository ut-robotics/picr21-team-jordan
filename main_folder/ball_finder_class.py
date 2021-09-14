"""
using classes can be uselful in the future with state machine alghorritm.
Btw, this isnt final resutl, just raw example
"""

import time
import cv2
import numpy as np
import imutils
import input_manager

class BallFinder():
    def __init__(self, color_type):
        self.path = "/home/jordan_team/picr21-team-jordan/main_folder/"
        self.fps = 0

        self.color_type = color_type
        self.cap = cv2.VideoCapture(4)
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cv2.namedWindow('Original')
        cv2.namedWindow('Thresh')

        default_values = input_manager.get_default_values(self.path)
        self.target_window = 'Thresh'
        cv2.createTrackbar('hl', self.target_window, default_values[0], 255, self.updateValue)
        cv2.createTrackbar('sl', self.target_window, default_values[1], 255, self.updateValue)
        cv2.createTrackbar('vl', self.target_window, default_values[2], 255, self.updateValue)
        cv2.createTrackbar('hh', self.target_window, default_values[3], 255, self.updateValue)
        cv2.createTrackbar('sh', self.target_window, default_values[4], 255, self.updateValue)
        cv2.createTrackbar('vh', self.target_window, default_values[5], 255, self.updateValue)
        cv2.createTrackbar('closing1', self.target_window,12, 100, self.updateValue)
        cv2.createTrackbar('closing2', self.target_window, 11, 100, self.updateValue)
        cv2.createTrackbar('dilation1', self.target_window,5, 100, self.updateValue)
        cv2.createTrackbar('dilation2', self.target_window, 4, 100, self.updateValue)

        self.blobparams = cv2.SimpleBlobDetector_Params()
        self.blobparams.filterByArea = True
        self.blobparams.minArea = 500
        self.blobparams.maxArea = 999999
        self.blobparams.filterByInertia = False
        self.blobparams.filterByConvexity = False
        self.detector = cv2.SimpleBlobDetector_create(self.blobparams)


    def updateValue(self, new_value):
        self.trackbar_value = new_value

    def track_ball_using_imutils(self, inspected_frame, target_frame):
        """
        white ball and black background
        use betwise_not(mask) if needed
        """
        cnts = cv2.findContours(inspected_frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # To see the centroid clearly
            if radius > 10:
                cv2.circle(target_frame, (int(x), int(y)), int(radius), (0, 255, 255), 5)
                cv2.circle(target_frame, center, 5, (0, 0, 255), -1)
                cv2.putText(target_frame, str(round(x)) + " : " + str(round(y)), (int(x), int(y - radius - 10)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)


    def track_ball_using_blob(self, inspected_frame, target_frame):
        """
        black ball and whiite background
        use betwise_not(mask) if needed
        """
        keypoints = self.detector.detect(inspected_frame)
        cv2.drawKeypoints(inspected_frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        if len(keypoints) > 0:
            for keypoint in keypoints:
                self.text = str(int(keypoint.pt[0])) + " : " + str(int(keypoint.pt[1]))
                self.x, self.y = int(keypoint.pt[0]), int(keypoint.pt[1])
                cv2.putText(target_frame, self.text, (self.x, self.y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    
    def main(self):
        while True:
            start_time = time.time()
            ret, frame = self.cap.read()


            hsv = cv2.cvtColor(frame, self.color_type)
            hsv_blured = cv2.medianBlur(hsv, 5)
            hl = cv2.getTrackbarPos('hl', self.target_window)
            sl = cv2.getTrackbarPos('sl', self.target_window)
            vl = cv2.getTrackbarPos('vl', self.target_window)
            hh = cv2.getTrackbarPos('hh', self.target_window)
            sh = cv2.getTrackbarPos('sh', self.target_window)
            vh = cv2.getTrackbarPos('vh', self.target_window)
            closing1 = cv2.getTrackbarPos('closing1', self.target_window)
            closing2 = cv2.getTrackbarPos('closing2', self.target_window)
            dilation1 = cv2.getTrackbarPos('dilation1', self.target_window)
            dilation2 = cv2.getTrackbarPos('dilation2', self.target_window)

            kernel1 = np.ones((closing1, closing2),np.uint8)
            kernel2 = np.ones((dilation1, dilation2),np.uint8)
            lowerLimits = np.array([hl, sl, vl])
            upperLimits = np.array([hh, sh, vh])

            mask = cv2.inRange(hsv_blured, lowerLimits, upperLimits)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel1)
            mask = cv2.dilate(mask, kernel2, iterations = 1)
            mask = cv2.bitwise_not(mask)

            self.track_ball_using_imutils(mask, frame)
            self.track_ball_using_blob(mask, frame)



            cv2.putText(frame, str(self.fps), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('Original', frame)
            cv2.imshow('Thresh', mask)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                input_manager.save_default_values(self.path, [str(x) for x in [hl, sl, vl, hh, sh, vh]])
                break
            
        

            self.fps = round(1.0 / (time.time() - start_time), 2)

        self.cap.release()
        cv2.destroyAllWindows()


    


ball_finder = BallFinder(cv2.COLOR_BGR2HSV)
ball_finder.main()