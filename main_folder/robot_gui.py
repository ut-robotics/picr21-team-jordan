import cv2
import numpy as np

import constants as const


class RobotGui:
    def __init__(self):
        self.color_image: np.ndarray = None
        self.masked_image_ball: np.ndarray = None
        self.masked_image_basket: np.ndarray = None
        self.fps = -1
        self.ball_info = [-1, -1, -1]
        self.current_state = "Initial"

        cv2.namedWindow(const.ORIGINAL_WINDOW)
        cv2.namedWindow(const.MASKED_WINDOW)
        cv2.namedWindow(const.MASKED_WINDOW_BASKET)

    def update_info(self, fps, current_state, ball_info, basket_info):
        self.fps = fps
        self.current_state = current_state
        self.ball_info = ball_info
        self.basket_info = basket_info

    def update_image(self, color_image, masked_image_ball, masked_image_basket):
        self.color_image = color_image
        self.masked_image_ball = masked_image_ball
        self.masked_image_ball = masked_image_basket

    def show_gui(self):
        ball_x = int(self.ball_info[0])
        ball_y = int(self.ball_info[1])
        ball_size = int(self.ball_info[2])
        center = self.ball_info[3]

        basket_x = int(self.ball_info[0])
        basket_y = int(self.ball_info[1])
        basket_size = int(self.basket_info[2])

        cv2.line(self.color_image, (const.CENTER_RANGE[0], 0), (const.CENTER_RANGE[0], const.HEIGHT), (0, 0, 0), 3)
        cv2.line(self.color_image, (const.CENTER_RANGE[-1], 0), (const.CENTER_RANGE[-1], const.HEIGHT), (0, 0, 0), 3)
        cv2.putText(self.color_image, str(self.fps), (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(self.color_image, "State: " + str(self.current_state), (120, 45), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        if ball_size > const.MIN_BALL_RADIUS:
                cv2.circle(self.color_image, (ball_x, ball_x), ball_size, (0, 255, 255), 5)
                cv2.circle(self.color_image, center, 5, (0, 0, 255), -1)
                cv2.putText(self.color_image, str(round(ball_x)) + " : " + str(round(ball_y)), (int(ball_x), int(ball_y - ball_size - 10)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # show image
        cv2.imshow(const.ORIGINAL_WINDOW, self.color_image)
        cv2.imshow(const.MASKED_WINDOW, self.masked_image_ball)
        cv2.imshow(const.MASKED_WINDOW_BASKET, self.masked_image_basket)

    def kill_gui(self):
        cv2.destroyAllWindows()
