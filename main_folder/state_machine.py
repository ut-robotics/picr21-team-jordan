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

    def run_current_state(self, ball_x, ball_y, ball_size):
        # if referee_command: TODO referee commands
        #     self.state = int(referee_command)

        if self.state == State.FIND_BALL:
            self.find_a_ball(ball_x, ball_y, ball_size)

        # print(f"State: {self.current_state}, Ball=(x:{ball_x}|size:{ball_size}, Action: {action}")
        return self.state

    def find_a_ball(self, ball_x, ball_y, ball_size):
        print(ball_x, ball_y, ball_size)
        """State.FIND_BALL action"""
        if ball_x == -1:
            self.Robot.move_robot(0, 0, 10)

        elif ball_x < const.CENTER_RANGE[0]:
            self.Robot.move_robot(0, 0, 5)

        elif ball_x > const.CENTER_RANGE[-1]:
            self.Robot.move_robot(0, 0, -5)

        # elif ball_x in const.CENTER_RANGE:
        #     if ball_size <= const.BALL_SIZE_TO_STOP:
        #         self.Robot.move_robot(state="transition", moving_direction=0, speed_limit=const.ROBOT_SPEED)
        #     else:
        #         self.Robot.move_robot(moving_direction=0, speed_limit=0)
        #         # self.current_state = "ball_found"
