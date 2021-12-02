import threading
import time

import cv2

from camera import RealsenseCamera
from enums import GameObject, State
from image_processor import ImageProcessor
from robot_gui import RobotGui
from socket_data_getter import SocketDataGetter
from state_machine import StateMachine


class Main:
    """
    Main class. Gets frames, applyes image processing to get:
    Ball coord, ball size, basket coord, basket size. Sends values to the StateMachine class
    """

    def __init__(self, enable_gui):
        self.enable_gui = enable_gui
        self.gui = RobotGui() if enable_gui else None
        self.state_machine = StateMachine()
        self.cam = RealsenseCamera()
        self.cam.open()
        self.image_processor = ImageProcessor(self.cam)

        # TODO implement referee command
        self.target_basket = GameObject.BASKET_BLUE
        self.my_robot_id = -1

        self.fps = 0
        self.current_state = State.INITIAL

    def main(self, socket_data):
        while True:
            start_time = time.time()

            # check for referee commands
            if socket_data:
                referee_command = socket_data.pop(0)  # TODO implement referee command interrupt of the current state
            else:
                referee_command = None

            # detect all objects
            aligned_depth = True if self.current_state == State.THROW else False
            results = self.image_processor.process_frame(aligned_depth=aligned_depth)
            ball_x, ball_y, ball_radius = -1, -1, -1
            basket_x, basket_y, basket_radius = -1, -1, -1

            if results.balls:
                ball = results.balls[0]
                ball_x = ball.x
                ball_y = ball.y
                ball_radius = int(ball.width / 2)

            basket_dist = -1
            basket = results.basket_b if self.target_basket == GameObject.BASKET_BLUE else results.basket_m
            if basket.exists:
                basket_x = basket.x
                basket_y = basket.y
                basket_radius = int(basket.width / 2)
                basket_dist = int(round(basket.distance*100))
            # print(basket_dist)
        
            # run robot
            self.current_state = self.state_machine.run_current_state(ball_x, ball_y, basket_x, basket_dist)
            
            # show gui
            if self.enable_gui:
                ball_info = [ball_x, ball_y, ball_radius, (ball_x, ball_y)]
                basket_info = [basket_x, basket_y, basket_radius, (basket_x, basket_y)]

                self.gui.update_image(results.color_frame)
                self.gui.update_info(self.fps, self.current_state, ball_info, basket_info)
                self.gui.show_gui()

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            self.fps = round(1.0 / (time.time() - start_time), 2)

        self.cam.close()

        if self.enable_gui:
            self.gui.kill_gui()


def socket_data_getter(out_q):
    # camera_image = SocketDataGetter()
    # camera_image.main(out_q)
    pass


def image_getter(in_q):
    try:
        state_machine = Main(enable_gui=True)
        state_machine.main(in_q)
    finally:
        state_machine.cam.close()


if __name__ == "__main__":
    socket_q = []
    image_q = []
    t1 = threading.Thread(target=socket_data_getter, args=(socket_q,))
    t1.daemon = True
    t3 = threading.Thread(target=image_getter, args=(socket_q,))
    t1.start()
    t3.start()
