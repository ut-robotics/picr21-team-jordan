import cv2
import numpy as np

import constants as const


class RobotGui:
    def __init__(self):
        self.color_image: np.ndarray = None
        self.fps = -1
        self.ball_info = [-1, -1, -1]
        self.basket_info = [-1, -1, -1]
        self.current_state = ""

        cv2.namedWindow(const.ORIGINAL_WINDOW)

    def update_info(self, fps, current_state, ball_info, basket_info):
        self.fps = fps
        self.current_state = current_state
        self.ball_info = ball_info
        self.basket_info = basket_info

    def update_image(self, color_image):
        self.color_image = color_image

    def show_gui(self):
        ball_x = int(self.ball_info[0])
        ball_y = int(self.ball_info[1])
        ball_raius = int(self.ball_info[2])
        center = self.ball_info[3]

        basket_x = int(self.basket_info[0])
        basket_y = int(self.basket_info[1])
        basket_size = int(self.basket_info[2])

        # draw all game info
        cv2.line(self.color_image, (const.CENTER_RANGE_X[0], 0), (const.CENTER_RANGE_X[0], const.HEIGHT_RESIZED), (0, 0, 0), 3)
        cv2.line(self.color_image, (const.CENTER_RANGE_X[-1], 0), (const.CENTER_RANGE_X[-1], const.HEIGHT_RESIZED), (0, 0, 0), 3)
        cv2.line(self.color_image, (0, const.CENTER_RANGE_Y[0]), (const.WIDTH_RESIZED, const.CENTER_RANGE_Y[0]), (0, 0, 0), 3)
        cv2.line(self.color_image, (0, const.CENTER_RANGE_Y[-1]), (const.WIDTH_RESIZED, const.CENTER_RANGE_Y[-1]), (0, 0, 0), 3)
        cv2.putText(self.color_image, str(self.fps), (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(self.color_image, "State: " + str(self.current_state), (120, 45), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        if ball_raius > const.MIN_BALL_RADIUS_TO_DETECT:
            cv2.circle(self.color_image, (ball_x, ball_x), ball_raius, (0, 255, 255), 5)
            cv2.circle(self.color_image, center, 5, (0, 0, 255), -1)
            cv2.putText(self.color_image, str(round(ball_x)) + " : " + str(round(ball_y)), (int(ball_x), int(ball_y - ball_raius - 10)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # show image
        try:
            cv2.imshow(const.ORIGINAL_WINDOW, self.color_image)
        except Exception:  # cv2.error: (-215:Assertion failed) size.width>0 && size.height>0 in function 'imshow'
            pass

    def kill_gui(self):
        cv2.destroyAllWindows()
