import cv2
import numpy as np

import constants as const


class RobotGui:
    def __init__(self):
        self.color_image: np.ndarray = None
        self.masked_image: np.ndarray = None
        self.fps = -1
        self.ball_info = [-1, -1, -1]
        self.current_state = "Initial"

        cv2.namedWindow(const.ORIGINAL_WINDOW)
        cv2.namedWindow(const.MASKED_WINDOW)
        
    def update_info(self, fps, current_state, ball_info):
        self.fps = fps
        self.current_state = current_state
        self.ball_info = ball_info

    def update_image(self, color_image, masked_image):
        self.color_image = color_image
        self.masked_image = masked_image

    def show_gui(self):
        ball_x = int(self.ball_info[0])
        ball_y = int(self.ball_info[1])
        ball_size = int(self.ball_info[2])
        
        cv2.line(self.color_image, (const.CENTER_RANGE[0], 0), (const.CENTER_RANGE[0], const.HEIGHT), (0, 0, 0), 3)
        cv2.line(self.color_image, (const.CENTER_RANGE[-1], 0), (const.CENTER_RANGE[-1], const.HEIGHT), (0, 0, 0), 3)
        cv2.putText(self.color_image, str(self.fps), (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(self.color_image, "State: " + str(self.current_state), (120, 45), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        text = str(ball_x) + " : " + str(ball_y) + ":::" + str(round(ball_size))
        cv2.putText(self.color_image, text, (ball_x, ball_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        # show image
        cv2.imshow(const.ORIGINAL_WINDOW, self.color_image)
        cv2.imshow(const.MASKED_WINDOW, self.masked_image)

    def kill_gui(self):
        cv2.destroyAllWindows()
