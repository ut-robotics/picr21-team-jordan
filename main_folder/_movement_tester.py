import cv2

import constants as const
from camera import Camera
from image_processing import ImageProcessing
from robot_movement import RobotMovement
from my_enums import Object

"""
Trowing test. Will help with calculating throwing function, that depends of the distance to the basket.
"""

if __name__ == "__main__":
    Robot = RobotMovement()
    Cam = Camera()
    ImageProcess = ImageProcessing()
    
    for _ in range(999):
        color_image = Cam.get_rgb_frame()
        color_image = cv2.resize(color_image, (const.WIDTH_RESIZED, const.HEIGHT_RESIZED))
        depth_image = Cam.get_depth_frame()
        mask_image_basket = ImageProcess.get_masked_image(color_image, Object.BASKET_BLUE)

        basket_x, basket_y, basket_radius, basket_center = ImageProcess.get_obj_coords(mask_image_basket)
        distance_to_basket = ImageProcess.get_distance_to_basket(depth_image, basket_center)
        print(distance_to_basket)

        Robot.move_robot_XY(0, 0, 0, 500)
