import time

import cv2
import numpy as np

from image_calibration import ImageCalibraion
from state_machine import StateMachine


class ImageGetter(ImageCalibraion):
    def __init__(self, enable_pyrealsense=False, enable_gui=True):  # TODO keep gui separate form other code
        super(ImageGetter, self).__init__(enable_pyrealsense=enable_pyrealsense)
        self.enable_gui = enable_gui

        if self.enable_gui:
            cv2.namedWindow(self.original_window)
            cv2.namedWindow(self.mask_window)

        self.CENTER_OFFSET = 70
        self.CENTER_RANGE = range(1)
        self.BALL_X, self.BALL_Y, self.BALL_SIZE = -1, -1, -1  # TODO ball_y is not required, but delete this later, im not sure
        self.MINIMAL_BALL_SIZE_TO_DETECT = 30
        self.BASKET_X, self.BASKET_Y, self.BASKET_SIZE = -1, -1, -1  # TODO basket_y is not required, but delete this later, im not sure
        self.REFEREE_COMMAND = "No command received"

        self.state_machine = StateMachine()

    def get_ball_coordinates(self, inspected_frame, target_frame):
        """returns coordinates of the biggest ball"""
        keypoints: list = self.detector.detect(inspected_frame)
        if self.enable_gui:
            cv2.drawKeypoints(inspected_frame, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # detect all keypoints
        kp_sizes = []
        if len(keypoints) > 0:
            for keypoint in keypoints:
                kp_sizes.append(keypoint.size)
                if keypoint.size > self.MINIMAL_BALL_SIZE_TO_DETECT:
                    x, y = int(keypoint.pt[0]), int(keypoint.pt[1])
                    text = str(round(x)) + " : " + str(round(y)) + ":::" + str(round(keypoint.size))

        # detects biggest keypoint, marks with different color
        try:
            biggest_keypoint = keypoints[kp_sizes.index(max(kp_sizes))]
            x, y, size = int(biggest_keypoint.pt[0]), int(biggest_keypoint.pt[1]), biggest_keypoint.size
            return int(round(x)), int(round(y)), int(round(size))
        except ValueError:
            return 0, 0, 0

    def get_basket_coordinates(self, inspected_frame, target_frame):
        """returns coordinates of the basket"""
        # TODO implement basket finding
        return -1, -1, -1

    def draw_info(self, frame):
        """draws information about game on original frame"""
        cv2.line(frame, (self.CENTER_RANGE[0], 0), (self.CENTER_RANGE[0], self.HEIGHT), (0, 0, 0), 3)
        cv2.line(frame, (self.CENTER_RANGE[-1], 0), (self.CENTER_RANGE[-1], self.HEIGHT), (0, 0, 0), 3)
        cv2.putText(frame, str(self.FPS), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, "State: " + self.REFEREE_COMMAND, (120, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        text = str(self.BALL_X) + " : " + str(self.BALL_Y) + ":::" + str(round(self.BALL_SIZE))
        cv2.putText(frame, text, (self.BALL_X, self.BALL_Y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    def main(self, socket_data):
        while True:
            start_time = time.time()

            # check for referee coommands
            if socket_data:
                self.REFEREE_COMMAND = socket_data.pop(0)  # TODO implement referee command interrurpt of the curent state

            # get camera images
            if self.enable_pyrealsense:
                color_image, depth_image = self.get_frame_using_pyrealsense()  # TODO do something with depth
                mask_image = self.apply_image_processing(color_image)
            else:
                _, color_image = self.cap.read()
                mask_image = self.apply_image_processing(color_image)

            # calculate range, that robot can consider as center X coord (X +/- offset)
            self.WIDTH = mask_image.shape[1]
            self.HEIGHT = mask_image.shape[0]
            self.CENTER_RANGE = range(int(self.WIDTH / 2) - self.CENTER_OFFSET, int(self.WIDTH / 2) + self.CENTER_OFFSET, 1)

            # running robot depends of the ball and basket coords and sizes
            self.BALL_X, self.BALL_Y, self.BALL_SIZE = self.get_ball_coordinates(mask_image, color_image)
            self.state_machine.run_current_state(BALL_X=self.BALL_X, BALL_SIZE=self.BALL_SIZE, BASKET_X=self.BASKET_X, BASKET_SIZE=self.BASKET_SIZE)

            # show gui
            if self.enable_gui:
                self.draw_info(color_image)
                cv2.imshow(self.original_window, color_image)
                cv2.imshow(self.mask_window, mask_image)
                cv2.imshow(self.depth_window, depth_image) if self.enable_pyrealsense else -1

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            self.FPS = round(1.0 / (time.time() - start_time), 2)
            self.REFEREE_COMMAND = "No command received"  # we dont need it anymore, we use it one time to interrupt the current action

        self.cap.release() if not self.enable_pyrealsense else self.pipeline.stop()
        if self.enable_gui:
            cv2.destroyAllWindows()