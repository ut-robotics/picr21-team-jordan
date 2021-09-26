"""
using classes can be uselful in the future with state machine alghorritm.
Btw, this isnt final resutl, just raw example
"""

import os
import time

import cv2
import imutils
import numpy as np


class CameraImage:
    def __init__(self):
        self.path: str = os.path.abspath(os.getcwd()) + "/main_folder/"
        self.fps = 0
        self.color_type = cv2.COLOR_BGR2HSV
        self.cap = cv2.VideoCapture(0)
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
        self.default_values_ball: list = self.get_default_values(self.path, "trackbar_values_ball")
        self.default_values_basket: list = ["TBA"]

        cv2.namedWindow(self.original_window)
        cv2.namedWindow(self.mask_window)
        cv2.namedWindow(self.trackbar_window, cv2.WINDOW_NORMAL)

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

    def track_ball_using_blob(self, inspected_frame, target_frame):
        contours, _ = cv2.findContours(inspected_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.001 * cv2.arcLength(cnt, True), True)
            cv2.drawContours(target_frame, [approx], 0, (255, 0, 0), 2)

        keypoints: list = self.detector.detect(inspected_frame)
        cv2.drawKeypoints(
            inspected_frame, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
        )
        kp_sizes = []
        if len(keypoints) > 0:
            for keypoint in keypoints:
                kp_sizes.append(keypoint.size)
                if keypoint.size > 30:
                    text = (
                        str(round(keypoint.pt[0]))
                        + " : "
                        + str(round(keypoint.pt[1]))
                        + ":::"
                        + str(round(keypoint.size))
                    )
                    x, y = int(keypoint.pt[0]), int(keypoint.pt[1])
                    cv2.putText(target_frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    def main(self):
        cv2.createTrackbar("hl", self.trackbar_window, self.default_values_ball[0], 255, self.update_value)
        cv2.createTrackbar("sl", self.trackbar_window, self.default_values_ball[1], 255, self.update_value)
        cv2.createTrackbar("vl", self.trackbar_window, self.default_values_ball[2], 255, self.update_value)
        cv2.createTrackbar("hh", self.trackbar_window, self.default_values_ball[3], 255, self.update_value)
        cv2.createTrackbar("sh", self.trackbar_window, self.default_values_ball[4], 255, self.update_value)
        cv2.createTrackbar("vh", self.trackbar_window, self.default_values_ball[5], 255, self.update_value)
        cv2.createTrackbar("closing1", self.trackbar_window, self.default_values_ball[6], 100, self.update_value)
        cv2.createTrackbar("closing2", self.trackbar_window, self.default_values_ball[7], 100, self.update_value)
        cv2.createTrackbar("dilation1", self.trackbar_window, self.default_values_ball[8], 100, self.update_value)
        cv2.createTrackbar("dilation2", self.trackbar_window, self.default_values_ball[9], 100, self.update_value)
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
            clos1 = cv2.getTrackbarPos("closing1", self.trackbar_window)
            clos2 = cv2.getTrackbarPos("closing2", self.trackbar_window)
            dil1 = cv2.getTrackbarPos("dilation1", self.trackbar_window)
            dil2 = cv2.getTrackbarPos("dilation2", self.trackbar_window)

            kernel1 = np.ones((clos1, clos2), np.uint8)
            kernel2 = np.ones((dil1, dil2), np.uint8)
            lowerLimits = np.array([hl, sl, vl])
            upperLimits = np.array([hh, sh, vh])

            mask = cv2.inRange(hsv_blured, lowerLimits, upperLimits)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel1)
            mask = cv2.dilate(mask, kernel2, iterations=1)
            mask = cv2.bitwise_not(mask)

            self.track_ball_using_blob(mask, frame)

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
