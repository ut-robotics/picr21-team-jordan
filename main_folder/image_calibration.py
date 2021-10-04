import os
import time

import cv2
import numpy as np


class CameraImage:
    def __init__(self):
        self.path: str = os.path.abspath(os.getcwd()) + "/main_folder/"
        self.default_values_ball: list = self.get_default_values(self.path, "trackbar_values_ball")
        self.default_values_basket: list = ["TBA"]
        
        self.color_type = cv2.COLOR_BGR2HSV
        self.cap = cv2.VideoCapture(4)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.original_window = "Original"
        self.mask_window = "Thresh"
        self.trackbar_window = "Trackbar"
        
        self.blobparams = cv2.SimpleBlobDetector_Params()
        self.blobparams.filterByArea = True
        self.blobparams.minArea = 300
        self.blobparams.maxArea = 999999
        self.blobparams.filterByInertia = False
        self.blobparams.filterByConvexity = False
        self.detector = cv2.SimpleBlobDetector_create(self.blobparams)
        self.fps = 0

    def update_value(self, new_value):
        self.trackbar_value = new_value

    def get_default_values(self, path, file_name):
        try:
            with open(path + file_name, "r") as file:
                return [int(x) for x in file.read().split()]
        except FileNotFoundError:
            return [127, 127, 127, 255, 255, 255, 1, 1, 1, 1]

    def save_default_values(self, path, file_name, array):
        with open(path + file_name, "w") as file:
            file.write(" ".join(array))

    def main(self):
        cv2.namedWindow(self.original_window)
        cv2.namedWindow(self.mask_window)
        cv2.namedWindow(self.trackbar_window, cv2.WINDOW_NORMAL)
        cv2.createTrackbar("hl", self.trackbar_window, self.default_values_ball[0], 255, self.update_value)
        cv2.createTrackbar("sl", self.trackbar_window, self.default_values_ball[1], 255, self.update_value)
        cv2.createTrackbar("vl", self.trackbar_window, self.default_values_ball[2], 255, self.update_value)
        cv2.createTrackbar("hh", self.trackbar_window, self.default_values_ball[3], 255, self.update_value)
        cv2.createTrackbar("sh", self.trackbar_window, self.default_values_ball[4], 255, self.update_value)
        cv2.createTrackbar("vh", self.trackbar_window, self.default_values_ball[5], 255, self.update_value)
        cv2.createTrackbar("clos1", self.trackbar_window, self.default_values_ball[6], 100, self.update_value)
        cv2.createTrackbar("clos2", self.trackbar_window, self.default_values_ball[7], 100, self.update_value)
        cv2.createTrackbar("dil1", self.trackbar_window, self.default_values_ball[8], 100, self.update_value)
        cv2.createTrackbar("dil2", self.trackbar_window, self.default_values_ball[9], 100, self.update_value)
        
        while True:
            start_time = time.time()
            _, frame = self.cap.read()

            hsv = cv2.cvtColor(frame, self.color_type)
            hsv_blured = cv2.medianBlur(hsv, 5)
            hl = cv2.getTrackbarPos("hl", self.trackbar_window)
            sl = cv2.getTrackbarPos("sl", self.trackbar_window)
            vl = cv2.getTrackbarPos("vl", self.trackbar_window)
            hh = cv2.getTrackbarPos("hh", self.trackbar_window)
            sh = cv2.getTrackbarPos("sh", self.trackbar_window)
            vh = cv2.getTrackbarPos("vh", self.trackbar_window)
            clos1 = cv2.getTrackbarPos("clos1", self.trackbar_window)
            clos2 = cv2.getTrackbarPos("clos2", self.trackbar_window)
            dil1 = cv2.getTrackbarPos("dil1", self.trackbar_window)
            dil2 = cv2.getTrackbarPos("dil2", self.trackbar_window)

            kernel1 = np.ones((clos1, clos2), np.uint8)
            kernel2 = np.ones((dil1, dil2), np.uint8)
            lowerLimits = np.array([hl, sl, vl])
            upperLimits = np.array([hh, sh, vh])

            mask = cv2.inRange(hsv_blured, lowerLimits, upperLimits)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel1)
            mask = cv2.dilate(mask, kernel2, iterations=1)
            mask = cv2.bitwise_not(mask)

            cv2.putText(frame, str(self.fps), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Original", frame)
            cv2.imshow("Thresh", mask)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.save_default_values(
                    self.path,
                    "trackbar_values_ball",
                    [str(x) for x in [hl, sl, vl, hh, sh, vh, dil1, dil2, clos1, clos2]],
                )
                break

            self.fps = round(1.0 / (time.time() - start_time), 2)

        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    camera_image = CameraImage()
    camera_image.main()
