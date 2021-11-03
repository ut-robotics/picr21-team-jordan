import cv2
import numpy as np

import constants as const


class RobotGui:
    def __init__(self):
        self.color_image: np.ndarray = None
        self.masked_image: np.ndarray = None
        self.fps = -1

        cv2.namedWindow(const.ORIGINAL_WINDOW)
        cv2.namedWindow(const.MASKED_WINDOW)

    def update_image(self, color_image, masked_image):
        self.color_image = color_image
        self.masked_image = masked_image

    def show_gui(self):
        # draw info
        cv2.line(self.color_image, (const.CENTER_RANGE[0], 0), (const.CENTER_RANGE[0], const.HEIGHT), (0, 0, 0), 3)
        cv2.line(self.color_image, (const.CENTER_RANGE[-1], 0), (const.CENTER_RANGE[-1], const.HEIGHT), (0, 0, 0), 3)
        cv2.putText(self.color_image, str(self.fps), (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        # cv2.putText(self.color_image, "State: " + str(self.current_state), (120, 45), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        # text = str(self.ball_x) + " : " + str(self.ball_y) + ":::" + str(round(self.ball_size))
        # cv2.putText(self.color_image, text, (self.ball_x, self.ball_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        # show image
        cv2.imshow(const.ORIGINAL_WINDOW, self.color_image)
        cv2.imshow(const.MASKED_WINDOW, self.masked_image)

    def kill_gui(self):
        cv2.destroyAllWindows()
