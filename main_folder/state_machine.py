from my_enums import State
import constants as const
from robot_movement import RobotMovement


class StateMachine:
    """
    This class manipulates robot depends of the ball and the basket coords.
    Manipulating is mechanical (turn left, right, etc.)
    """

    def __init__(self):
        self.Robot = RobotMovement()
        self.state: int = State.FIND_BALL

    def run_current_state(self, ball_x, ball_y, ball_radius, basket_x, basket_radius):
        # if referee_command: TODO referee commands
        #     self.state = int(referee_command)

        if self.state == State.FIND_BALL:
            self.find_a_ball(ball_x)

        if self.state == State.GET_TO_BALL:
            self.get_to_ball(ball_x, ball_y, ball_radius)

        if self.state == State.CENTER_BASKET_AND_BALL:
            self.center_a_basket(ball_x, ball_y, ball_radius, basket_x, basket_radius)

        return self.state

    def find_a_ball(self, ball_x):
        """State.FIND_BALL action"""
        if ball_x == -1:
            self.Robot.move_robot_XY(0, 0, 15)
        else:
            self.state = State.GET_TO_BALL

    def get_to_ball(self, ball_x, ball_y, ball_radius):
        """State.GET_TO_BALL action"""
        robot_speed_rot = self.calculate_rotation_speed(ball_x)
        robot_speed_y = self.calculate_y_speed(ball_y)

        if ball_y == -1:
            self.state = State.FIND_BALL
        # TODO check if last condition is compulsory
        elif ball_y in const.CENTER_RANGE_Y and ball_radius > const.MIN_BALL_RADIUS_TO_STOP:
            self.state = State.CENTER_BASKET_AND_BALL
        else:
            self.Robot.move_robot_XY(0, robot_speed_y, robot_speed_rot)

    def center_a_basket(self, ball_x, ball_y, ball_radius, basket_x, basket_radius):
        # TODO ADD FIND A BASKET STATE
        robot_speed_rot = self.calculate_rotation_speed(ball_x)
        robot_speed_y = self.calculate_y_speed(ball_y)
        robot_x = self.calculate_x_speed(basket_x)

        if ball_y == -1:
            self.state = State.FIND_BALL
        # TODO check if last condition is compulsory
        elif ball_y in const.CENTER_RANGE_Y and ball_radius > const.MIN_BALL_RADIUS_TO_STOP and basket_x in const.CENTER_RANGE_BASKET and ball_radius > const.MINIMAL_BASKET_RADIUS_TO_DETECT:
            self.Robot.move_robot_XY(0, 0, 0)
        else:
            self.Robot.move_robot_XY(robot_x, robot_speed_y, robot_speed_rot)

    def limit_speed(self, speed):
        return max(min(speed, const.MAXIMUM_SPEED), -const.MAXIMUM_SPEED)

    def calculate_rotation_speed(self, ball_x):
        speed_rot = (const.CENTER_X - ball_x) / const.ROT_MULTIPLIER
        return self.limit_speed(speed_rot)

    def calculate_y_speed(self, ball_y):
        speed_y = ((const.CENTER_Y - ball_y) / const.Y_MULTIPLIER) ** 3
        return self.limit_speed(speed_y)

    def calculate_x_speed(self, basket_x):
        speed_x = (const.CENTER_X - basket_x) / const.X_MULTIPLIER
        return speed_x
