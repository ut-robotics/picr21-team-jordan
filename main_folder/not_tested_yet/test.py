import math



def move_robot_XY(self, speed_x=0, speed_y=0, rotation_speed=0, thrower_speed=0):
        def degrees(rad):
            return(rad * 180 / math.pi )

        pure_deg = degrees(math.atan2(speed_y, speed_x))
        moving_direction = 0
        if speed_x > 0 and speed_y > 0:
            moving_direction = 90 + pure_deg 
        if speed_x > 0 and speed_y < 0: 
            moving_direction = 90 - pure_deg
        if speed_x < 0 and speed_y < 0:
            moving_direction = 270 + pure_deg
        if speed_x < 0 and speed_y > 0:
            moving_direction = 180 + pure_deg
        print(moving_direction)
        
move_robot_XY(1, 10, 15)