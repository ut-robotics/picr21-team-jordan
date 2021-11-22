from enum import Enum

import constants as const
from robot_movement import RobotMovement


class State(Enum):
    INITIAL = 0
    FIND_BALL = 1
    GET_TO_BALL = 2
    FIND_BASKET = 3
    CENTER_BASKET_AND_BALL = 4


class StateMachine:
    """
    This class manipulates robot depends of the ball and the basket coords.
    Manipulating is mechanical (turn left, right, etc.)
    """

    def __init__(self):
        self.Robot = RobotMovement()
        self.state: int = State.FIND_BALL

    def run_current_state(self, ball_x, ball_y, ball_radius):
        # if referee_command: TODO referee commands
        #     self.state = int(referee_command)

        if self.state == State.FIND_BALL:
            self.find_a_ball(ball_x)

        if self.state == State.GET_TO_BALL:
            self.get_to_ball(ball_x, ball_y, ball_radius)

        return self.state

    def find_a_ball(self, ball_x):
        """State.FIND_BALL action"""
        # robot_speed_rot = self.calculate_rotation_speed(ball_x)

        if ball_x == -1:
            self.Robot.move_robot_XY(0, 0, 15)

        # elif ball_x in const.CENTER_RANGE_X:
        #     self.Robot.move_robot_XY(0, 0, 0)
        #     self.state = State.GET_TO_BALL

        else:
            # self.Robot.move_robot_XY(0, 0, robot_speed_rot)
            self.state = State.GET_TO_BALL

    def get_to_ball(self, ball_x, ball_y, ball_radius):
        """State.GET_TO_BALL action"""
        robot_speed_rot = self.calculate_rotation_speed(ball_x)
        robot_speed_y = self.calculate_y_speed(ball_y)

        if ball_y == -1:
            self.state = State.FIND_BALL
        elif ball_y in const.CENTER_RANGE_Y and ball_radius > const.MIN_BALL_RADIUS_TO_STOP:
            self.Robot.move_robot_XY(0, 0, 0)
        else:
            self.Robot.move_robot_XY(0, robot_speed_y, robot_speed_rot)

    def limit_speed(self, speed):
        return max(min(speed, const.MAXIMUM_SPEED), -const.MAXIMUM_SPEED)

    def calculate_rotation_speed(self, ball_x):
        """Rot speed is linear dependence"""
        speed_rot = int((const.CENTER_X - ball_x) / const.ROT_MULTIPLIER)
        return self.limit_speed(speed_rot)

    def calculate_y_speed(self, ball_y):
        """Y speed speed is linear dependence"""
        speed_y = int(((const.CENTER_Y - ball_y) / const.Y_MULTIPLIER) ** 3)
        return self.limit_speed(speed_y)

    def calculate_x_speed(self, tba):
        pass
