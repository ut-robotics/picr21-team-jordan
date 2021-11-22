import cv2
import imutils
import numpy as np

import constants as const
import file_manager


class ImageProcessing:
    def __init__(self) -> None:
        self.default_values_ball = file_manager.get_default_values(const.CONFIG_PATH, const.BALL)
        self.default_values_basket_blue = file_manager.get_default_values(const.CONFIG_PATH, const.BASKET_BLUE)
        self.default_values_basket_rose = file_manager.get_default_values(const.CONFIG_PATH, const.BASKET_ROSE)

        self.blobparams = cv2.SimpleBlobDetector_Params()
        self.blobparams.minDistBetweenBlobs = const.MIN_DISTANCE_BETWEEN_BLOBS
        self.blobparams.filterByArea = True
        self.blobparams.minArea = const.BLOB_MIN_AREA
        self.blobparams.maxArea = const.BLOB_MAX_AREA
        self.blobparams.filterByInertia = False
        self.blobparams.filterByConvexity = False
        self.blobparams.filterByCircularity = False
        self.detector = cv2.SimpleBlobDetector_create(self.blobparams)

        self.color_type = cv2.COLOR_BGR2HSV

    def get_masked_image(self, frame, type, default_values=[]):
        if not default_values:
            if type == const.BALL:
                default_values = self.default_values_ball
            elif type == const.BASKET_BLUE:
                default_values = self.default_values_basket_blue
            elif type == const.BASKET_ROSE:
                default_values = self.default_values_basket_rose

        lowerLimits = np.array([default_values[0], default_values[1], default_values[2]])
        upperLimits = np.array([default_values[3], default_values[4], default_values[5]])
        kernel1 = np.ones((default_values[6], default_values[7]), np.uint8)
        kernel2 = np.ones((default_values[8], default_values[9]), np.uint8)

        frame = cv2.cvtColor(frame, self.color_type)
        frame = cv2.medianBlur(frame, const.BLUR)
        mask = cv2.inRange(frame, lowerLimits, upperLimits)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel1)
        mask = cv2.dilate(mask, kernel2, iterations=1)

        return mask

    def get_ball_coords(self, mask_image):
        cnts = cv2.findContours(mask_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
            if radius <= const.MIN_BALL_RADIUS_TO_DETECT:
                return -1, -1, -1, -1
            else:
                return int(round(x)), int(round(y)), int(round(radius)), center
        else:
            return -1, -1, -1, -1

    def get_biggest_blob_coords(self, inspected_frame):
        keypoints: list = self.detector.detect(inspected_frame)
        # detect all keypoints
        kp_sizes = []
        if len(keypoints) > 0:
            for keypoint in keypoints:
                kp_sizes.append(keypoint.size)
                if keypoint.size > const.MINIMAL_BALL_SIZE_TO_DETECT:
                    x, y = int(keypoint.pt[0]), int(keypoint.pt[1])

        # detects biggest keypoint
        try:
            biggest_keypoint = keypoints[kp_sizes.index(max(kp_sizes))]
            x, y, size = int(biggest_keypoint.pt[0]), int(biggest_keypoint.pt[1]), biggest_keypoint.size
            return int(round(x)), int(round(y)), int(round(size))
        except ValueError:
            return -1, -1, -1
