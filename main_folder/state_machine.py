import move_function as hw
import constants as const

BALL_SIZE_TO_STOP = 70
ROBOT_SPEED = 25


class StateMachine:
    """
    This class manipulates robot depends of the ball and the basket coords.
    Manipulating is mechanical (turn left, right, etc.)
    """

    def __init__(self):
        self.states = ["find_a_ball", "get closer to a ball"] #TODO implement actual states
        self.current_state_index = 0

    def next_state(self):
        self.current_state_index += 1
        if self.current_state_index == len(self.states):
            self.current_state_index = 0

    def run_current_state(self, ball_x, ball_size, basket_x, basket_size):
        # working code
        """
        if ball_x == -1:
            action = "Can't see a ball"
            hw.move_robot(moving_direction=0, speed_limit=0)
        elif ball_x < const.CENTER_RANGE[0]:
            action = "Turning left"
            hw.move_robot(moving_direction=-90, speed_limit=ROBOT_SPEED)
        elif ball_x > const.CENTER_RANGE[-1]:
            action = "Turning right"
            hw.move_robot(moving_direction=90, speed_limit=ROBOT_SPEED)
        elif ball_x in const.CENTER_RANGE:
            if ball_size <= BALL_SIZE_TO_STOP:
                action = "moving forward"
                hw.move_robot(moving_direction=90, speed_limit=ROBOT_SPEED)
            else:
                action = "stop"
                hw.move_robot(moving_direction=0, speed_limit=0)
        """

        if self.state[self.current_state_index] == "find_a ball":
            action = "looking for a ball"
            self.find_a_ball()

        elif self.state[self.current_state_index] == "follow_a_ball":
            action = ""
            self.find_a_ball()

        elif self.state[self.current_state_index] == "grab_a_ball ball":
            self.find_a_ball()

        elif self.state[self.current_state_index] == "grab_a_ball ball":
            self.find_a_ball()

        print(f"State: {self.states[self.current_state_index]}, Ball=(x:{ball_x}|size:{ball_size}, Action = {action})")
        return action
        # TODO add actual states and refactor this stairs mess
