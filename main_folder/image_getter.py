import threading
import time

import cv2
import numpy as np

import constants as const
from image_calibration import ImageCalibraion
from robot_gui import RobotGui
from socket_data_getter import SocketDataGetter
from state_machine import StateMachine


class ImageGetter(ImageCalibraion):
    """
    Main class. Gets frames, apllyes image processing to get:
    Ball coord, ball size, basket coord, basket size.
    Sends values to the StateMahchine class
    """

    def __init__(self, enable_gui):
        super(ImageGetter, self).__init__()
        self.enable_gui = enable_gui

        self.Gui = RobotGui() if enable_gui else None
        self.State_machine = StateMachine()

    def get_biggest_blob_coord(self, inspected_frame):
        """returns coordinates of the biggest ball"""
        keypoints: list = self.detector.detect(inspected_frame)
        if self.enable_gui:
            cv2.drawKeypoints(inspected_frame, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # detect all keypoints
        kp_sizes = []
        if len(keypoints) > 0:
            for keypoint in keypoints:
                kp_sizes.append(keypoint.size)
                if keypoint.size > const.MINIMAL_BALL_SIZE_TO_DETECT:
                    x, y = int(keypoint.pt[0]), int(keypoint.pt[1])

        # detects biggest keypoint, marks with different color
        try:
            biggest_keypoint = keypoints[kp_sizes.index(max(kp_sizes))]
            x, y, size = int(biggest_keypoint.pt[0]), int(biggest_keypoint.pt[1]), biggest_keypoint.size
            return int(round(x)), int(round(y)), int(round(size))
        except ValueError:
            return -1, -1, -1

    def main(self, socket_data):
        while True:
            start_time = time.time()

            # check for referee coommands
            if socket_data:
                referee_command = socket_data.pop(0)  # TODO implement referee command interrurpt of the curent state
            else:
                referee_command = None

            # get camera images
            color_image, depth_image = self.get_frame_using_pyrealsense()  # TODO do something with depth
            mask_image_ball = self.apply_image_processing(color_image, const.BALL)
            mask_image_basket = self.apply_image_processing(color_image, const.BASKET_BLUE)

            # running robot depends of the ball and basket coords and sizes
            ball_x, ball_y, ball_radius, center = self.track_ball_using_imutils(mask_image_ball) #TODO size = radius
            basket_x, basket_y, basket_size = self.get_biggest_blob_coord(mask_image_basket)
            self.current_state = self.State_machine.run_current_state(ball_x, ball_y, ball_radius)

            # show gui
            if self.enable_gui:
                ball_info = [ball_x, ball_y, ball_radius, center] 
                basket_info = [basket_x, basket_y, basket_size]

                self.Gui.update_info(self.fps, self.current_state, ball_info, basket_info)
                self.Gui.update_image(color_image)
                self.Gui.show_gui()
                
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            self.fps = round(1.0 / (time.time() - start_time), 2)

        self.pipeline.stop()

        if self.enable_gui:
            self.Gui.kill_gui()


def socket_data_getter(out_q):
    camera_image = SocketDataGetter()
    camera_image.main(out_q)


def image_getter(in_q):
    state_machine = ImageGetter(enable_gui=True)
    state_machine.main(in_q)


if __name__ == "__main__":
    socket_q = []
    image_q = []
    t1 = threading.Thread(target=socket_data_getter, args=(socket_q,))
    t1.daemon = True
    t3 = threading.Thread(target=image_getter, args=(socket_q,))
    t1.start()
    t3.start()
