import threading
import time

import cv2

from camera import RealsenseCamera
from enums import GameObject, State
from image_processor import ImageProcessor
from manual_control import ManualController
from robot_movement import RobotMovement
from state_machine import MAXIMUM_SPEED, ROT_MULTIPLIER

if __name__ == "__main__":
    cam = RealsenseCamera()
    image_processor = ImageProcessor(cam)

    cam.open()

    robot = RobotMovement()
    manual_controller = ManualController()
    manual_controller.main()
    try:
        while True:
            robot.move_robot_XY(0, 0, 0, manual_controller.speed_throw)
            results = image_processor.process_frame(aligned_depth=True)
            CENTER_X = int(848 / 2)
            basket_dist = -1
            basket_x = 0
            if results.basket_m.exists:
                basket = results.basket_m
                basket_dist = int(round(basket.distance*100))
            print(manual_controller.speed_throw, basket_dist)
    finally:
        cam.close()      
    
    
