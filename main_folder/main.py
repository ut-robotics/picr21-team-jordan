import threading
import time

import cv2

import constants as const
from camera import Camera
from image_processing import ImageProcessing
from robot_gui import RobotGui
from socket_data_getter import SocketDataGetter
from state_machine import StateMachine


class Main:
    """
    Main class. Gets frames, applyes image processing to get:
    Ball coord, ball size, basket coord, basket size.
    Sends values to the StateMachine class
    """

    def __init__(self, enable_gui):
        self.enable_gui = enable_gui
        self.Gui = RobotGui() if enable_gui else None
        self.StateMachine = StateMachine()
        self.Cam = Camera()
        self.ImageProcess = ImageProcessing()

        # TODO implement referee command
        self.target_basket = const.BASKET_BLUE
        self.my_robot_id = -1

        self.fps = 0

    def main(self, socket_data):
        while True:
            start_time = time.time()

            # check for referee commands
            if socket_data:
                referee_command = socket_data.pop(0)  # TODO implement referee command interrupt of the current state
            else:
                referee_command = None

            # get camera images
            color_image = self.Cam.get_rgb_frame()
            color_image = cv2.resize(color_image, (const.WIDTH_RESIZED, const.HEIGHT_RESIZED))
            depth_image = self.Cam.get_depth_frame()
            mask_image_ball = self.ImageProcess.get_masked_image(color_image, const.BALL)
            mask_image_basket = self.ImageProcess.get_masked_image(color_image, self.target_basket)

            # running robot depends of the ball and basket coords and sizes
            ball_x, ball_y, ball_radius, ball_center = self.ImageProcess.get_obj_coords(mask_image_ball)
            basket_x, basket_y, basket_radius, basket_center = self.ImageProcess.get_obj_coords(mask_image_basket)
            self.current_state = self.StateMachine.run_current_state(ball_x, ball_y, ball_radius)
            print(basket_radius)
            # show gui
            if self.enable_gui:
                ball_info = [ball_x, ball_y, ball_radius, ball_center]
                basket_info = [basket_x, basket_y, basket_radius, basket_center]

                self.Gui.update_info(self.fps, self.current_state, ball_info, basket_info)
                self.Gui.update_image(color_image)
                self.Gui.show_gui()

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            self.fps = round(1.0 / (time.time() - start_time), 2)

        self.Cam.stop_camera()

        if self.enable_gui:
            self.Gui.kill_gui()


def socket_data_getter(out_q):
    camera_image = SocketDataGetter()
    camera_image.main(out_q)


def image_getter(in_q):
    state_machine = Main(enable_gui=True)
    state_machine.main(in_q)


if __name__ == "__main__":
    socket_q = []
    image_q = []
    t1 = threading.Thread(target=socket_data_getter, args=(socket_q,))
    t1.daemon = True
    t3 = threading.Thread(target=image_getter, args=(socket_q,))
    t1.start()
    t3.start()
