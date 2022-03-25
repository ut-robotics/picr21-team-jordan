import pyrealsense2 as rs
import numpy as np


RGB_WIDTH = 848
RGB_HEIGHT = 480
RGB_FPS = 60
EXPOSURE = 50
WHITE_BALANCE = 3500
DEPTH_WIDTH, DEPTH_HEIGHT, DEPTH_FPS = RGB_WIDTH, RGB_HEIGHT, RGB_FPS


class RealsenseCamera:
    """
    Camera class, can be replaced if you don't have realsense camera.
    Returns frames (rgb and depth) as np arrays.
    """
    def __init__(self, depth_enabled=True):
        self.rgb_height = RGB_HEIGHT
        self.rgb_width = RGB_WIDTH
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, RGB_WIDTH, RGB_HEIGHT, rs.format.bgr8, RGB_FPS)
        self.depth_enabled = depth_enabled
        self.align = rs.align(rs.stream.color)
        self.depth_scale = -1
        if self.depth_enabled:
            self.config.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, DEPTH_FPS)

    def open(self):
        profile = self.pipeline.start(self.config)
        color_sensor = profile.get_device().query_sensors()[1]
        color_sensor.set_option(rs.option.enable_auto_exposure, False)
        color_sensor.set_option(rs.option.enable_auto_white_balance, False)
        color_sensor.set_option(rs.option.white_balance, WHITE_BALANCE)
        color_sensor.set_option(rs.option.exposure, EXPOSURE)
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

    def get_color_frame(self):
        frames = self.pipeline.wait_for_frames()
        return np.asanyarray(frames.get_color_frame().get_data())

    def has_depth_capability(self) -> bool:
        return self.depth_enabled

    def get_frames(self, aligned=False):
        """
        Returns rgb frame and depth frame
        """
        frames = self.pipeline.wait_for_frames()
        if aligned:
            frames = self.align.process(frames)
        return np.asanyarray(frames.get_color_frame().get_data()), np.asanyarray(frames.get_depth_frame().get_data())

    def close(self):
        self.pipeline.stop()
