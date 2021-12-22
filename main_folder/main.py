import json
import threading
import time

import cv2
import numpy as np

from camera import RealsenseCamera
from enums import GameObject, State
from image_processor import ImageProcessor
from robot_gui import RobotGui
from socket_data_getter import SocketDataGetter
from state_machine import StateMachine
from manual_control import ManualController


class Main:
    """
    Main class. Gets frames, applyes image processing to get:
    Ball coord, ball size, basket coord, basket size. Sends values to the StateMachine class
    """

    def __init__(self, enable_gui):
        self.cam = RealsenseCamera()
        self.cam.open()
        self.gui = RobotGui() if enable_gui else None
        self.state_machine = StateMachine()
        self.image_processor = ImageProcessor(self.cam)
        self.manual_controller = ManualController()

        self.target_basket = GameObject.BASKET_BLUE
        self.robot_id = "001TRT"
        self.current_state = State.INITIAL
        self.run = False

        self.enable_gui = enable_gui
        self.fps = 0

    def main(self, socket_data):
        manual_controller = self.manual_controller
        manual_controller.main()
        while True:
            start_time = time.time()

            # check if manual control is enabled
            if manual_controller.is_enabled:
                manual_controller.robot.move_robot_XY(manual_controller.speed_x, manual_controller.speed_y, manual_controller.speed_rot)
            else:
                # check for referee commands
                if socket_data:
                    command = socket_data.pop(0)
                    try:
                        if command["signal"] == "start":
                            index = command["targets"].index(self.robot_id)
                            target_basket = command["baskets"][index]
                            self.target_basket = GameObject.BASKET_BLUE if target_basket == "blue" else GameObject.BASKET_ROSE
                            self.run = True

                        elif command["signal"] == "stop":
                            index = command["targets"].index(self.robot_id)
                            self.run = False
                    except ValueError:
                        # target isn't our robot id
                        pass

                print(self.run, self.target_basket)

                # detect all objects
                # aligned_depth = True if self.current_state == State.THROW else False
                aligned_depth = True
                results = self.image_processor.process_frame(aligned_depth=aligned_depth)
                ball_x, ball_y, ball_radius = -1, -1, -1
                basket_x, basket_y, basket_radius = -1, -1, -1
                basket_dist = -1

                if results.balls:
                    ball = results.balls[0]
                    ball_x = ball.x
                    ball_y = ball.y
                    ball_radius = int(ball.width / 2)

                    basket = results.basket_b if self.target_basket == GameObject.BASKET_BLUE else results.basket_m
                    if basket.exists:
                        basket_x = basket.x
                        basket_y = basket.y
                        basket_radius = int(basket.width / 2)
                        basket_dist = int(round(basket.distance * 100))

                # run robot
                if self.run:
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
    socket_data_getter = SocketDataGetter("localhost", 9999)
    socket_data_getter.main(out_q)


def image_getter(in_q):
    try:
        state_machine = Main(enable_gui=True)
        state_machine.main(in_q)
    finally:
        if state_machine is not None:
            state_machine.cam.close()


if __name__ == "__main__":
    socket_q = []
    image_q = []
    t1 = threading.Thread(target=socket_data_getter, args=(socket_q,))
    t1.daemon = True
    t3 = threading.Thread(target=image_getter, args=(socket_q,))
    t1.start()
    t3.start()
