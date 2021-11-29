import cv2
import numpy as np

import constants as const
from enums import ColorRGB, Window


class RobotGui:
    def __init__(self):
        self.color_image: np.ndarray = None
        self.fps = -1
        self.ball_info = [-1, -1, -1]
        self.basket_info = [-1, -1, -1]
        self.current_state = ""

        cv2.namedWindow(Window.ORIGINAL)

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
        ball_radius = int(self.ball_info[2])
        ball_center = self.ball_info[3]

        basket_x = int(self.basket_info[0])
        basket_y = int(self.basket_info[1])
        basket_radius = int(self.basket_info[2])
        basket_center = self.basket_info[3]

        # draw all game info
        cv2.line(self.color_image, (const.CENTER_RANGE_X[0], 0), (const.CENTER_RANGE_X[0], const.HEIGHT), ColorRGB.GREEN, 3)
        cv2.line(self.color_image, (const.CENTER_RANGE_X[-1], 0), (const.CENTER_RANGE_X[-1], const.HEIGHT), ColorRGB.GREEN, 3)
        cv2.line(self.color_image, (0, const.CENTER_RANGE_Y[0]), (const.WIDTH, const.CENTER_RANGE_Y[0]), ColorRGB.GREEN, 3)
        cv2.line(self.color_image, (0, const.CENTER_RANGE_Y[-1]), (const.WIDTH, const.CENTER_RANGE_Y[-1]), ColorRGB.GREEN, 3)
        cv2.line(self.color_image, (const.CENTER_RANGE_BASKET[0], 0), (const.CENTER_RANGE_BASKET[0], const.HEIGHT), ColorRGB.BLUE, 3)
        cv2.line(self.color_image, (const.CENTER_RANGE_BASKET[-1], 0), (const.CENTER_RANGE_BASKET[-1], const.HEIGHT), ColorRGB.BLUE, 3)

        cv2.putText(self.color_image, str(self.fps), (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 1, ColorRGB.BLACK, 2)
        cv2.putText(self.color_image, str(self.current_state), (120, 45), cv2.FONT_HERSHEY_SIMPLEX, 1, ColorRGB.BLACK, 2)
        cv2.circle(self.color_image, (ball_x, ball_y), 5, ColorRGB.GREEN, 5)
        cv2.circle(self.color_image, ball_center, 5, ColorRGB.GREEN, -1)
        cv2.putText(self.color_image, str(round(ball_x)) + " : " + str(round(ball_y)), (int(ball_x), int(ball_y - ball_radius - 10)), cv2.FONT_HERSHEY_SIMPLEX, 1, ColorRGB.GREEN, 2)
        cv2.circle(self.color_image, basket_center, 5, ColorRGB.BLUE, -1)
        cv2.rectangle(self.color_image, (basket_x - basket_radius, basket_y - basket_radius), (basket_x + basket_radius, basket_y + basket_radius), ColorRGB.BLUE, 3)

        # show image
        try:
            cv2.imshow(Window.ORIGINAL, self.color_image)
        # cv2.error: (-215:Assertion failed) size.width>0 && size.height>0 in function 'imshow'
        except Exception:
            pass

    def kill_gui(self):
        cv2.destroyAllWindows()
