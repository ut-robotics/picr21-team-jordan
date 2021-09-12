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
        cv2.createTrackbar('closing1', self.target_window,12, 100, self.updateValue)
        cv2.createTrackbar('closing2', self.target_window, 11, 100, self.updateValue)
        cv2.createTrackbar('dilation1', self.target_window,5, 100, self.updateValue)
        cv2.createTrackbar('dilation2', self.target_window, 4, 100, self.updateValue)


    def updateValue(self, new_value):
        self.trackbar_value = new_value
        
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
            mask_inverted = cv2.bitwise_not(mask)
            closing = cv2.morphologyEx(mask_inverted, cv2.MORPH_CLOSE, kernel1)
            processed_image = cv2.dilate(closing,kernel2,iterations = 1)





            cv2.putText(frame, str(self.fps), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('Original', frame)
            cv2.imshow('Thresh', processed_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                input_manager.save_default_values(self.path, [str(x) for x in [hl, sl, vl, hh, sh, vh]])
                break
            
        

            self.fps = round(1.0 / (time.time() - start_time), 2)

        self.cap.release()
        cv2.destroyAllWindows()

ball_finder = BallFinder(cv2.COLOR_BGR2HSV)
ball_finder.main()