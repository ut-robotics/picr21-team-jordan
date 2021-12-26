from enums import Position, State
import constants as const
from robot_movement import RobotMovement

MAXIMUM_SPEED = 45
ROT_MULTIPLIER = 10
Y_MULTIPLIER = 5
X_MULTIPLIER = 10
THROW_COUNTER = 50


class StateMachine:
    """
    This class manipulates robot depends of the ball and the basket coords.
    Manipulating is mechanical (turn left, right, etc.)
    """

    def __init__(self, robot_movement):
        self.robot_movement = robot_movement
        self.state = State.FIND_BALL
        self.last_basket_pos = Position.LEFT
        self.counter = 0

    def run_current_state(self, ball_x, ball_y, basket_x, basket_distance):

        if basket_x != -1:
            self.update_last_basket_pos(basket_x)

        if self.state == State.FIND_BALL:
            self.find_a_ball(ball_x)

        elif self.state == State.GET_TO_BALL:
            self.get_to_ball(ball_x, ball_y)

        elif self.state == State.FIND_A_BASKET:
            self.find_a_basket(ball_x, ball_y, basket_x)

        elif self.state == State.THROW:
            self.throw_a_ball(basket_distance, basket_x)

        return self.state

    def update_last_basket_pos(self, basket_x):
        self.last_basket_pos = Position.LEFT if basket_x < const.CENTER_X else Position.RIGHT

    def find_a_ball(self, ball_x):
        """State action"""
        if ball_x == -1:
            self.robot_movement.move_robot_XY(0, 0, 15)
        else:
            self.state = State.GET_TO_BALL

    def get_to_ball(self, ball_x, ball_y):
        """State action"""
        robot_speed_rot = self.calculate_rotation_speed(ball_x)
        robot_speed_x = self.calculate_x_speed(ball_x)
        robot_speed_y = self.calculate_y_speed(ball_y)

        if ball_x == -1:
            self.state = State.FIND_BALL
        elif ball_y in const.CENTER_RANGE_Y and ball_x in const.CENTER_RANGE_X:
            self.state = State.FIND_A_BASKET
        else:
            self.robot_movement.move_robot_XY(robot_speed_x, robot_speed_y, robot_speed_rot)

    def find_a_basket(self, ball_x, ball_y, basket_x):
        """State action"""
        robot_speed_rot = self.calculate_rotation_speed(ball_x)
        robot_speed_y = self.calculate_y_speed(ball_y)
        robot_speed_x = self.calculate_x_speed(basket_x)

        if ball_y == -1:
            self.state = State.FIND_BALL
        if basket_x == -1:
            robot_speed_x == MAXIMUM_SPEED if self.last_basket_pos == Position.LEFT else -MAXIMUM_SPEED

        if ball_x in const.CENTER_RANGE_X and basket_x in const.CENTER_RANGE_BASKET:
            self.robot_movement.move_robot_XY(0, 0, 0)
            self.state = State.THROW
        else:
            self.robot_movement.move_robot_XY(robot_speed_x, robot_speed_y, robot_speed_rot)

    def throw_a_ball(self, basket_distance, basket_x):
        self.counter += 1

        thrower_speed = self.calculate_thrower_speed(basket_distance)
        robot_speed_y = int(MAXIMUM_SPEED / 3)
        robot_speed_rot = self.calculate_rotation_speed(basket_x) if basket_x != -1 else 0

        self.robot_movement.move_robot_XY(0, robot_speed_y, robot_speed_rot, thrower_speed)
        if self.counter > THROW_COUNTER:
            self.counter = 0
            self.state = State.FIND_BALL

    def calculate_thrower_speed(self, distance):
        thrower_speed = int(2.9859 * distance + 700)
        return thrower_speed

    def limit_speed(self, speed):
        return max(min(speed, MAXIMUM_SPEED), -MAXIMUM_SPEED)

    def calculate_rotation_speed(self, ball_x):
        speed_rot = (const.CENTER_X - ball_x) / ROT_MULTIPLIER
        return self.limit_speed(speed_rot)

    def calculate_y_speed(self, ball_y):
        speed_y = ((const.CENTER_Y - ball_y) / Y_MULTIPLIER) ** 3
        return self.limit_speed(speed_y)

    def calculate_x_speed(self, basket_x):
        speed_x = (const.CENTER_X - basket_x) / X_MULTIPLIER
        return speed_x
