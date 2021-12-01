import threading
import time

import cv2

from camera import RealsenseCamera
from enums import GameObject, State
from image_processor import ImageProcessor
from robot_movement import RobotMovement
from state_machine import MAXIMUM_SPEED

if __name__ == "__main__":
    cam = RealsenseCamera()
    image_processor = ImageProcessor(cam)
    
    cam.open()
    results = image_processor.process_frame(aligned_depth=True)
    
    basket_dist = -1
    basket_x = 0
    if results.basket_m.exists:
        basket = results.basket_m
        basket_dist = int(round(basket.distance*100))
        basket_x = basket.x
    print(basket_dist)
    cam.close()
    
    speed = int(input("Speed: "))
    robot = RobotMovement()
    for _ in range(25000):
        basket_dist = -1
        basket_x = 0
        if results.basket_m.exists:
            basket = results.basket_m
            basket_dist = int(round(basket.distance*100))
            basket_x = basket.x
            robot.move_robot_XY(0, MAXIMUM_SPEED, int((464 - basket_x) / 10), speed)
    
    is_goal = int(input("Is goal? (1/0): "))  
    
    if is_goal == 1:
        with open("thrower.txt", "a") as f:
            f.write(str(speed) + " " + str(basket_dist) + "\n")
    # 125=1070-90
    
    
