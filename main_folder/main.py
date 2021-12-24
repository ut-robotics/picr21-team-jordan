import asyncio
import json
import time

import cv2
from websockets import connect

from camera import RealsenseCamera
from enums import GameObject, State
from image_processor import ImageProcessor
from manual_control import ManualController
from robot_gui import RobotGui
from state_machine import StateMachine


class GameLogic:
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

        self.target_basket = GameObject.BASKET_BLUE
        self.robot_id = "001TRT"
        self.current_state = State.INITIAL
        self.run = False

        self.enable_gui = enable_gui
        self.fps = 0

    def main(self, socket_data, manual_controller):
        start_time = time.time()

        # check if manual control is enabled
        if manual_controller.enable:
            manual_controller.robot.move_robot_XY(manual_controller.speed_x, manual_controller.speed_y, manual_controller.speed_rot, manual_controller.speed_throw)
        # enable game logic
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

            print(f"Run: {self.run} Basket: {self.target_basket[-4:]}")
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
        self.fps = round(1.0 / (time.time() - start_time), 2)


async def run_listener(out_q):
    async with connect("ws://localhost:8888") as websocket:
        while True:
            server_data = await websocket.recv()
            command = json.loads(server_data)
            out_q.append(command)


async def run_game_logic(in_q):
    game_logic = GameLogic(enable_gui=True)
    manual_controller = ManualController()
    manual_controller.main()
    while True:
        game_logic.main(in_q, manual_controller)
        await asyncio.sleep(0.0001)
        if cv2.waitKey(1) & 0xFF == ord("x"):
            break
    game_logic.cam.close()
    if game_logic.enable_gui:
        game_logic.gui.kill_gui()


loop = asyncio.get_event_loop()
q = []
loop.create_task(run_game_logic(q))
loop.create_task(run_listener(q))
loop.run_forever()
