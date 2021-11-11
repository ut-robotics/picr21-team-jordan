import os
import time

import cv2
import numpy as np
import pyrealsense2 as rs

import constants as const

CAM_ID = 4
BLOB_MIN_AREA = 0
BLOB_MAX_AREA = 999_999
BLUR = 5
CONFIG_PATH = "/home/jordan_team/picr21-team-jordan/main_folder/config/"


class ImageCalibraion:
    """
    This class calibrates threshold values to properly see a ball and a basket and saves this value to /config file
    Only one main prupose -- generate config files
    """

    def __init__(self, enable_pyrealsense=False):
        self.enable_pyrealsense = enable_pyrealsense
        self.default_values_ball: list = self.get_default_values(CONFIG_PATH, "trackbar_values_ball")
        self.default_values_basket: list = ["TBA"]  # TODO

        if enable_pyrealsense:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, const.WIDTH_DEPTH, const.HEIGHT_DEPTH, rs.format.z16, 60)
            config.enable_stream(rs.stream.color, const.WIDTH, const.HEIGHT, rs.format.bgr8, 60)
            self.pipeline.start(config)
            self.alpha_depth = 0.9
        else:
            self.cap = cv2.VideoCapture(CAM_ID)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, const.WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, const.HEIGHT)
        self.color_type = cv2.COLOR_BGR2HSV

        self.blobparams = cv2.SimpleBlobDetector_Params()
        self.blobparams.filterByArea = True
        self.blobparams.minArea = BLOB_MIN_AREA
        self.blobparams.maxArea = BLOB_MAX_AREA
        self.blobparams.filterByInertia = False
        self.blobparams.filterByConvexity = False
        self.detector = cv2.SimpleBlobDetector_create(self.blobparams)

        self.fps = 0

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

    def apply_image_processing(self, frame, is_calibration=False):
        # change color space (RBG -> HSV)
        hsv = cv2.cvtColor(frame, self.color_type)
        hsv_blured = cv2.medianBlur(hsv, BLUR)

        # update trackbar values
        if is_calibration:
            for index, value in enumerate(["hl", "sl", "vl", "hh", "sh", "vh"]):
                self.default_values_ball[index] = cv2.getTrackbarPos(value, const.TRACKBAR_WINDOW)

        # update masked image
        lowerLimits = np.array([self.default_values_ball[0], self.default_values_ball[1], self.default_values_ball[2]])
        upperLimits = np.array([self.default_values_ball[3], self.default_values_ball[4], self.default_values_ball[5]])
        mask = cv2.inRange(hsv_blured, lowerLimits, upperLimits)
        mask = cv2.bitwise_not(mask)

        return mask

    def get_frame_using_pyrealsense(self):
        """Returns numpy array that represents the image"""
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=self.alpha_depth), cv2.COLORMAP_JET)

        return color_image, depth_colormap

    def main(self):
        # create gui
        cv2.namedWindow(const.ORIGINAL_WINDOW)
        cv2.namedWindow(const.MASKED_WINDOW)
        cv2.namedWindow(const.TRACKBAR_WINDOW, cv2.WINDOW_NORMAL)
        cv2.namedWindow(const.DEPTH_WINDOW) if self.enable_pyrealsense else -1
        for index, value in enumerate(["hl", "sl", "vl", "hh", "sh", "vh"]):
            cv2.createTrackbar(value, const.TRACKBAR_WINDOW, self.default_values_ball[index], 255, self.update_value)
        cv2.createTrackbar("alpha", const.TRACKBAR_WINDOW, 30, 100, self.update_value) if self.enable_pyrealsense else None

        while True:
            start_time = time.time()

            # get frames
            if self.enable_pyrealsense:
                color_image, depth_image = self.get_frame_using_pyrealsense()
                mask_image = self.apply_image_processing(color_image, is_calibration=True)  # TODO do something with depth
            else:
                _, color_image = self.cap.read()
                mask_image = self.apply_image_processing(color_image, is_calibration=True)

            # show frames
            cv2.putText(color_image, str(self.fps), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow(const.ORIGINAL_WINDOW, color_image)
            cv2.imshow(const.MASKED_WINDOW, mask_image)
            cv2.imshow(const.DEPTH_WINDOW, depth_image) if self.enable_pyrealsense else -1
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.save_default_values(CONFIG_PATH, "trackbar_values_ball", [str(x) for x in self.default_values_ball])
                break

            self.fps = round(1.0 / (time.time() - start_time), 2)
        self.cap.release() if not self.enable_pyrealsense else self.pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    camera_image = ImageCalibraion(enable_pyrealsense=True)
    camera_image.main()
