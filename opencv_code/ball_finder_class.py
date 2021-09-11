"""
using classes can be uselful in the future with state machine alghorritm.
Btw, this isnt final resutl, just raw example
"""

import time
import cv2
import numpy as np
import input_manager

class BallFinder():
    def __init__(self, color_type):
        self.path = "/home/jordan_team/picr21-team-jordan/opencv_code/"
        self.fps = 0

        self.color_type = color_type
        self.cap = cv2.VideoCapture(4)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
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

    def updateValue(self, new_value):
        self.trackbar_value = new_value
        
    def main(self):
        while True:
            start_time = time.time()
            ret, frame = self.cap.read()
            frame = cv2.cvtColor(frame, self.color_type)

            hl = cv2.getTrackbarPos('hl', self.target_window)
            sl = cv2.getTrackbarPos('sl', self.target_window)
            vl = cv2.getTrackbarPos('vl', self.target_window)
            hh = cv2.getTrackbarPos('hh', self.target_window)
            sh = cv2.getTrackbarPos('sh', self.target_window)
            vh = cv2.getTrackbarPos('vh', self.target_window)
            lowerLimits = np.array([hl, sl, vl])
            upperLimits = np.array([hh, sh, vh])
            thresh = cv2.inRange(frame, lowerLimits, upperLimits)
            thresh = cv2.bitwise_not(thresh)

            #outimage = cv2.bitwise_and(frame, frame, mask = thresh)
            
            cv2.putText(frame, str(self.fps), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('Original', frame)
            cv2.imshow('Thresh', thresh)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                input_manager.save_default_values(self.path, [str(x) for x in [hl, sl, vl, hh, sh, vh]])
                break
            
        

            self.fps = round(1.0 / (time.time() - start_time), 2)

        self.cap.release()
        cv2.destroyAllWindows()

ball_finder = BallFinder(cv2.COLOR_BGR2HSV)
ball_finder.main()