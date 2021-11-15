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
            self.find_a_ball(ball_x)
            
        if self.state == State.GET_TO_BALL:
            self.get_to_ball(ball_x, ball_y, ball_size)

        # print(f"State: {self.current_state}, Ball=(x:{ball_x}|size:{ball_size}, Action: {action}")
        return self.state

    def find_a_ball(self, ball_x):
        robot_speed_rot = int((const.CENTER_X - ball_x)/20)
        """State.FIND_BALL action"""
        if ball_x == -1:
            self.Robot.move_robot_XY(0, 0, 15)
            
        elif ball_x in const.CENTER_RANGE:
            self.Robot.move_robot_XY(0, 0, 0)
            self.state = State.GET_TO_BALL
        
        else:
            self.Robot.move_robot_XY(0, 0, robot_speed_rot)
        

    def get_to_ball(self, ball_x, ball_y, ball_size):
        # ball size 24 is close enough
        robot_speed_y = int((const.CENTER_Y - ball_y)/5)
        robot_speed_rot = int((const.CENTER_X - ball_x)/10)
        if ball_y == -1:
            self.state = State.FIND_BALL
        elif ball_y in const.CENTER_RANGE_Y:
            self.Robot.move_robot_XY(0, 0, 0)
        else:
            self.Robot.move_robot_XY(0, robot_speed_y, robot_speed_rot)
            
