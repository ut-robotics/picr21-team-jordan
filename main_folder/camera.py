import cv2
import numpy as np
import pyrealsense2 as rs

import constants as const


class Camera:
    def __init__(self) -> np.asanyarray:
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, const.WIDTH_DEPTH, const.HEIGHT_DEPTH, rs.format.z16, 60)
        config.enable_stream(rs.stream.color, const.WIDTH, const.HEIGHT, rs.format.bgr8, 60)
        self.profile = self.pipeline.start(config)
        self.color_sensor = self.profile.get_device().query_sensors()[1]
        self.color_sensor.set_option(rs.option.enable_auto_exposure, False)
        self.color_sensor.set_option(rs.option.enable_auto_white_balance, False)
        self.color_sensor.set_option(rs.option.white_balance, const.WHITE_BALANCE)
        self.color_sensor.set_option(rs.option.exposure, const.EXPOSURE)

    def get_depth_frame(self):
        """Returns numpy array that represents the depth image"""
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=const.ALPHA_DEPTH), cv2.COLORMAP_JET)
        depth_colormap_crop = depth_colormap[const.CROP_Y1 : const.CROP_Y1 + const.CROP_Y2, 1 : 0 + const.WIDTH - 1]

        return depth_colormap_crop

    def get_rgb_frame(self):
        """Returns numpy array that represents the rgb image"""
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        color_image_crop = color_image[const.CROP_Y1 : const.CROP_Y1 + const.CROP_Y2, 1 : 0 + const.WIDTH - 1]

        return color_image_crop

    def stop_camera(self):
        self.pipeline.stop()
