import threading
import time

import cv2

from camera import RealsenseCamera
from enums import GameObject, State
from image_processor import ImageProcessor
from robot_movement import RobotMovement
from state_machine import MAXIMUM_SPEED, ROT_MULTIPLIER

if __name__ == "__main__":
    cam = RealsenseCamera()
    image_processor = ImageProcessor(cam)
    
    cam.open()
    

    
    speed = int(input("Speed: "))
    robot = RobotMovement()
    counter = 0
    robot_speed_x = int(MAXIMUM_SPEED/3)
    dist = []
    
    while counter < 100:
        results = image_processor.process_frame(aligned_depth=True)
        counter += 1
        CENTER_X = int(848 / 2)
        basket_dist = -1
        basket_x = 0
        if results.basket_b.exists:
            basket = results.basket_b
            basket_dist = int(round(basket.distance*100))
            dist.append(basket_dist)
            basket_x = basket.x
            robot_speed_rot = (CENTER_X - basket_x) / ROT_MULTIPLIER
            robot.move_robot_XY(0, robot_speed_x, robot_speed_rot, speed)
    print(dist[0])
    is_goal = int(input("Is goal? (1/0): "))  
    
    if is_goal == 1:
        with open("thrower.txt", "a") as f:
            f.write(str(speed) + " " + str(basket_dist) + "\n")
            
    cam.close()
    
    
