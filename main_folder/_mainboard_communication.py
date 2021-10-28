import sys
import move_function as hw

"""
Mainboard communication task presentation
"""
robot_speed = int(sys.argv[1]) 
moving_direction = int(sys.argv[2])
thrower_speed = int(sys.argv[3])
iterations = int(sys.argv[4])
for i in range(iterations):
    hw.move_robot(moving_direction=moving_direction, speed_limit=robot_speed, thrower_speed=thrower_speed)
