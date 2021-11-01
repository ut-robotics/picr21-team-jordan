import threading
import time

import cv2
import numpy as np

import constants as const
from image_calibration import ImageCalibraion
from socket_data_getter import SocketDataGetter
from state_machine import StateMachine


class ImageGetter(ImageCalibraion):
    """
    Main class. Gets frames, apllyes image processing to get:
    Ball coord, ball size, basket coord, basket size.
    Sends values to the StateMahchine class
    """

    def __init__(self, enable_pyrealsense, enable_gui):  # TODO implement interfaces to separate the code
        super(ImageGetter, self).__init__(enable_pyrealsense=enable_pyrealsense)
        self.enable_gui = enable_gui

        if self.enable_gui:
            cv2.namedWindow(const.ORIGINAL_WINDOW)
            cv2.namedWindow(const.MASKED_WINDOW)

        self.ball_x, self.ball_y, self.ball_size = -1, -1, -1  # TODO ball_y is not required, but delete this later, im not sure
        self.basket_x, self.basket_y, self.basket_size = -1, -1, -1  # TODO basket_y is not required, but delete this later, im not sure

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
                    text = str(round(x)) + " : " + str(round(y)) + ":::" + str(round(keypoint.size))

        # detects biggest keypoint, marks with different color
        try:
            biggest_keypoint = keypoints[kp_sizes.index(max(kp_sizes))]
            x, y, size = int(biggest_keypoint.pt[0]), int(biggest_keypoint.pt[1]), biggest_keypoint.size
            return int(round(x)), int(round(y)), int(round(size))
        except ValueError:
            return -1, -1, -1

    def get_basket_coordinates(self, inspected_frame, target_frame):
        """returns coordinates of the basket"""
        # TODO implement basket finding
        return -1, -1, -1

    def draw_info(self, frame):
        """draws information about the game on original frame"""
        cv2.line(frame, (const.CENTER_RANGE[0], 0), (const.CENTER_RANGE[0], const.HEIGHT), (0, 0, 0), 3)
        cv2.line(frame, (const.CENTER_RANGE[-1], 0), (const.CENTER_RANGE[-1], const.HEIGHT), (0, 0, 0), 3)
        cv2.putText(frame, str(self.fps), (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, "State: " + self.current_state, (120, 45), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, "Action: " + self.current_action, (120, 75), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        text = str(self.ball_x) + " : " + str(self.ball_y) + ":::" + str(round(self.ball_size))
        cv2.putText(frame, text, (self.ball_x, self.ball_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

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
            self.ball_x, self.ball_y, self.ball_size = self.get_ball_coordinates(mask_image)
            self.current_state, self.current_action = self.state_machine.run_current_state(referee_command, self.ball_x, self.ball_size, self.basket_x, self.basket_size)

            # show gui
            if self.enable_gui:
                self.draw_info(color_image)
                cv2.imshow(const.ORIGINAL_WINDOW, color_image)
                cv2.imshow(const.MASKED_WINDOW, mask_image)
                cv2.imshow(const.DEPTH_WINDOW, depth_image) if self.enable_pyrealsense else -1

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            self.fps = round(1.0 / (time.time() - start_time), 2)

        self.cap.release() if not self.enable_pyrealsense else self.pipeline.stop()
        if self.enable_gui:
            cv2.destroyAllWindows()


def producer(out_q):
    camera_image = SocketDataGetter()
    camera_image.main(out_q)


def consumer(in_q):
    state_machine = ImageGetter(enable_pyrealsense=False, enable_gui=True)
    state_machine.main(in_q)


if __name__ == "__main__":
    q = []
    t1 = threading.Thread(target=producer, args=(q,))
    t1.daemon = True
    t2 = threading.Thread(target=consumer, args=(q,))
    t1.start()
    t2.start()
