"""
using classes can be uselful in the future with state machine alghorritm.
Btw, this isnt final resutl, just raw example
"""

from image_calibration import CameraImage
import time

import cv2
import numpy as np


class StateMachine(CameraImage):
    def __init__(self):
        super(StateMachine, self).__init__()

    def track_ball_using_blob(self, inspected_frame, target_frame):
        # for debbuging and finding blobs
        contours, _ = cv2.findContours(inspected_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.001 * cv2.arcLength(cnt, True), True)
            cv2.drawContours(target_frame, [approx], 0, (255, 0, 0), 2)

        keypoints: list = self.detector.detect(inspected_frame)
        cv2.drawKeypoints(
            inspected_frame, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
        )

        # detect all keypoints
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

        # detects biggest keypoint, marks with different color
        try:
            biggest_keypoint_index = kp_sizes.index(max(kp_sizes))
            biggest_keypoint = keypoints[biggest_keypoint_index]
            x, y = int(biggest_keypoint.pt[0]), int(biggest_keypoint.pt[1])
            text = (
                str(round(biggest_keypoint.pt[0]))
                + " : "
                + str(round(biggest_keypoint.pt[1]))
                + ":::"
                + str(round(biggest_keypoint.size))
            )
            cv2.putText(target_frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        except ValueError:
            pass

    def apply_image_processing(self, frame):
        hsv = cv2.cvtColor(frame, self.color_type)
        hsv_blured = cv2.medianBlur(hsv, 5)
        hl = self.default_values_ball[0]
        sl = self.default_values_ball[1]
        vl = self.default_values_ball[2]
        hh = self.default_values_ball[3]
        sh = self.default_values_ball[4]
        vh = self.default_values_ball[5]
        clos1 = self.default_values_ball[6]
        clos2 = self.default_values_ball[7]
        dil1 = self.default_values_ball[8]
        dil2 = self.default_values_ball[9]
        kernel1 = np.ones((clos1, clos2), np.uint8)
        kernel2 = np.ones((dil1, dil2), np.uint8)
        lowerLimits = np.array([hl, sl, vl])
        upperLimits = np.array([hh, sh, vh])
        mask = cv2.inRange(hsv_blured, lowerLimits, upperLimits)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel1)
        mask = cv2.dilate(mask, kernel2, iterations=1)
        mask = cv2.bitwise_not(mask)

        return mask

    def main(self):
        while True:
            start_time = time.time()
            _, frame = self.cap.read()
            mask = self.apply_image_processing(frame)

            self.track_ball_using_blob(mask, frame)

            cv2.putText(frame, str(self.fps), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Original", frame)
            cv2.imshow("Thresh", mask)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            self.fps = round(1.0 / (time.time() - start_time), 2)

        self.cap.release()
        cv2.destroyAllWindows()


camera_image = StateMachine()
camera_image.main()







"""
# CameraImage class:

def track_ball_using_imutils(self, inspected_frame, target_frame):
    inspected_frame = cv2.bitwise_not(inspected_frame)
    cnts = cv2.findContours(inspected_frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        try:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        except ZeroDivisionError:
            pass

        # To see the centroid clearly
        if radius > 10:
            cv2.circle(target_frame, (int(x), int(y)), int(radius), (0, 255, 255), 5)
            cv2.circle(target_frame, center, 5, (0, 0, 255), -1)
            cv2.putText(
                target_frame,
                str(round(x)) + " : " + str(round(y)),
                (int(x), int(y - radius - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2,
            )
"""