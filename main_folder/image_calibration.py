import os
import time
from collections import deque

import cv2
import imutils
import numpy as np
import pyrealsense2 as rs

import constants as const

CROP_Y1 = 170
CROP_Y2 = 350
CAM_ID = 4
BLOB_MIN_AREA = 0
BLOB_MAX_AREA = 999_999
MIN_DISTANCE_BETWEEN_BLOBS = 40
MIN_BALL_RADIUS = 1
BLUR = 5
CONFIG_PATH = "/home/jordan_team/picr21-team-jordan/main_folder/config/"


class ImageCalibraion:
    """
    This class calibrates threshold values to properly see a ball and a basket and saves this value to /config file
    Only one main prupose -- generate config files
    """

    def __init__(self):
        self.default_values_ball: list = self.get_default_values(CONFIG_PATH, "trackbar_values_ball")
        self.default_values_basket: list = self.get_default_values(CONFIG_PATH, "trackbar_values_basket")

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, const.WIDTH_DEPTH, const.HEIGHT_DEPTH, rs.format.z16, 60)
        config.enable_stream(rs.stream.color, const.WIDTH, const.HEIGHT, rs.format.bgr8, 60)
        # self.pipeline.start(config)
        self.profile = self.pipeline.start(config)
        self.color_sensor = self.profile.get_device().query_sensors()[1]
        self.color_sensor.set_option(rs.option.enable_auto_exposure, False)
        self.color_sensor.set_option(rs.option.enable_auto_white_balance, False)
        self.color_sensor.set_option(rs.option.white_balance, 3500)
        self.color_sensor.set_option(rs.option.exposure, 50)
        self.alpha_depth = 0.9

        self.color_type = cv2.COLOR_BGR2HSV

        self.blobparams = cv2.SimpleBlobDetector_Params()
        self.blobparams.minDistBetweenBlobs = MIN_DISTANCE_BETWEEN_BLOBS
        self.blobparams.filterByArea = True
        self.blobparams.minArea = BLOB_MIN_AREA
        self.blobparams.maxArea = BLOB_MAX_AREA
        self.blobparams.filterByInertia = False
        self.blobparams.filterByConvexity = False
        self.blobparams.filterByCircularity = False
        # self.blobparams.minCircularity = MIN_CIRCULARITY
        self.detector = cv2.SimpleBlobDetector_create(self.blobparams)

        self.fps = 0

        self.pts = deque(maxlen=64)

    def update_value(self, new_value):
        self.trackbar_value = new_value

    def get_default_values(self, path, file_name):
        """Looks for default threshold values for image processing in config file path"""
        try:
            with open(path + file_name, "r") as file:
                return [int(x) for x in file.read().split()]
        except FileNotFoundError:
            return [127, 127, 127, 255, 255, 255]

    def save_default_values(self, path, file_name, array):
        with open(path + file_name, "w") as file:
            file.write(" ".join(array))

    def get_frame_using_pyrealsense(self):
        """Returns numpy array that represents the image"""
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=self.alpha_depth), cv2.COLORMAP_JET)

        color_image = color_image[CROP_Y1 : CROP_Y1 + CROP_Y2, 0 : 0 + const.WIDTH]
        depth_colormap = depth_colormap[CROP_Y1 : CROP_Y1 + CROP_Y2, 0 : 0 + const.WIDTH]
        return color_image, depth_colormap

    def apply_image_processing(self, frame, type, is_calibration=False):
        # change color space (RBG -> HSV)
        hsv = cv2.cvtColor(frame, self.color_type)
        hsv_blured = cv2.medianBlur(hsv, BLUR)
        # hsv_blured = hsv

        # update trackbar values
        if is_calibration:
            for index, value in enumerate(["hl", "sl", "vl", "hh", "sh", "vh"]):
                if type == const.BALL:
                    self.default_values_ball[index] = cv2.getTrackbarPos(value, const.TRACKBAR_WINDOW)
                elif type == const.BASKET:
                    self.default_values_basket[index] = cv2.getTrackbarPos(value, const.TRACKBAR_WINDOW)

        # update masked image
        if type == const.BALL:
            lowerLimits = np.array([self.default_values_ball[0], self.default_values_ball[1], self.default_values_ball[2]])
            upperLimits = np.array([self.default_values_ball[3], self.default_values_ball[4], self.default_values_ball[5]])
        elif type == const.BASKET:
            lowerLimits = np.array([self.default_values_basket[0], self.default_values_basket[1], self.default_values_basket[2]])
            upperLimits = np.array([self.default_values_basket[3], self.default_values_basket[4], self.default_values_basket[5]])

        mask = cv2.inRange(hsv_blured, lowerLimits, upperLimits)
        mask = cv2.bitwise_not(mask)

        return mask

    def track_ball_using_imutils(self, inspected_frame):
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
            if radius <= const.MIN_BALL_RADIUS:
                return -1, -1, -1, -1
            else:
                return int(round(x)), int(round(y)), int(round(radius)), center
        else:
            return -1, -1, -1, -1

    def mainloop(self, type):
        if type == const.BALL:
            default_values = self.default_values_ball
        elif type == const.BASKET:
            default_values = self.default_values_basket

        # create gui
        cv2.namedWindow(const.ORIGINAL_WINDOW)
        cv2.namedWindow(const.MASKED_WINDOW)
        cv2.namedWindow(const.TRACKBAR_WINDOW, cv2.WINDOW_NORMAL)
        cv2.namedWindow(const.DEPTH_WINDOW)
        for index, value in enumerate(["hl", "sl", "vl", "hh", "sh", "vh"]):
            cv2.createTrackbar(value, const.TRACKBAR_WINDOW, default_values[index], 255, self.update_value)
        cv2.createTrackbar("alpha", const.TRACKBAR_WINDOW, 30, 100, self.update_value)

        # calibrartion for a ball
        while True:
            start_time = time.time()

            color_image, depth_image = self.get_frame_using_pyrealsense()
            mask_image = self.apply_image_processing(color_image, const.BALL, is_calibration=True)  # TODO do something with depth \
            # self.draw_keypoints(color_image, mask_image)
            x, y, radius, center = self.track_ball_using_imutils(mask_image)
            print(x, y, radius, center)
            # To see the centroid clearly
            if radius > const.MIN_BALL_RADIUS:
                cv2.circle(color_image, (int(x), int(y)), int(radius), (0, 255, 255), 5)
                cv2.circle(color_image, center, 5, (0, 0, 255), -1)
                cv2.putText(color_image, str(round(x)) + " : " + str(round(y)), (int(x), int(y - radius - 10)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # show frames
            cv2.putText(color_image, str(self.fps), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(color_image, type, (5, 55), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow(const.ORIGINAL_WINDOW, color_image)
            cv2.imshow(const.MASKED_WINDOW, mask_image)
            # cv2.imshow(const.DEPTH_WINDOW, depth_image)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                if type == const.BALL:
                    name = "trackbar_values_ball"
                elif type == const.BASKET:
                    name = "trackbar_values_basket"
                self.save_default_values(CONFIG_PATH, name, [str(x) for x in default_values])
                break
            self.fps = round(1.0 / (time.time() - start_time), 2)

    def main(self):
        self.mainloop(const.BALL)
        self.mainloop(const.BASKET)
        self.pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    camera_image = ImageCalibraion()
    camera_image.main()
