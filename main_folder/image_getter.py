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

    def __init__(self, enable_pyrealsense, enable_gui):
        super(ImageGetter, self).__init__(enable_pyrealsense=enable_pyrealsense)

        self.enable_gui = enable_gui
        self.Gui = RobotGui() if enable_gui else None

        self.state_machine = StateMachine()

    def get_ball_coordinates(self, inspected_frame):
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

    def get_basket_coordinates(self, inspected_frame):
        """returns coordinates of the basket"""
        # TODO implement basket finding
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
            if self.enable_pyrealsense:
                color_image, depth_image = self.get_frame_using_pyrealsense()  # TODO do something with depth
                mask_image = self.apply_image_processing(color_image)
            else:
                _, color_image = self.cap.read()
                mask_image = self.apply_image_processing(color_image)

            # running robot depends of the ball and basket coords and sizes
            ball_x, ball_y, ball_size = self.get_ball_coordinates(mask_image)
            basket_x, basket_y, basket_size = self.get_basket_coordinates(mask_image)
            self.current_state = self.state_machine.run_current_state(referee_command, ball_x, ball_size, basket_x, basket_size)

            # show gui
            if self.enable_gui:
                ball_info = [self.fps, ball_x, ball_y, ball_size] #TODO send info to gui
                self.Gui.update_info(self.fps, self.current_state, ball_info)
                self.Gui.update_image(color_image, mask_image)
                self.Gui.show_gui()
                

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            self.fps = round(1.0 / (time.time() - start_time), 2)

        self.cap.release() if not self.enable_pyrealsense else self.pipeline.stop()

        if self.enable_gui:
            self.Gui.kill_gui()


def socket_data_getter(out_q):
    camera_image = SocketDataGetter()
    camera_image.main(out_q)


def image_getter(in_q):
    state_machine = ImageGetter(enable_pyrealsense=True, enable_gui=True)
    state_machine.main(in_q)


if __name__ == "__main__":
    socket_q = []
    image_q = []
    t1 = threading.Thread(target=socket_data_getter, args=(socket_q,))
    t1.daemon = True
    t3 = threading.Thread(target=image_getter, args=(socket_q,))
    t1.start()
    t3.start()
