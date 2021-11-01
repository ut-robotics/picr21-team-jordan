import move_function as hw
import constants as const

BALL_SIZE_TO_STOP = 65
ROBOT_SPEED = 25


class StateMachine:
    """
    This class manipulates robot depends of the ball and the basket coords.
    Manipulating is mechanical (turn left, right, etc.)
    """

    def __init__(self):
        self.states = ["find_a_ball", "get_closer_to_a_ball"]  # TODO implement actual states
        self.current_state_index = 0
        self.current_state = "Initial"
        self.current_action = "Chillin'"
        self.prev_state = None

    def next_state(self):
        """Round-Robin state order from states list"""
        self.prev_state = self.current_state
        self.current_state_index += 1
        if self.current_state_index == len(self.states):
            self.current_state_index = 0
        self.current_state = self.states[self.current_state_index]

    def interrupt_robot(self, referee_command):
        """Gets referee command and makes current state equal referee commnad"""
        self.prev_state = self.current_state
        self.current_state = referee_command

    def run_current_state(self, referee_command, ball_x, ball_size, basket_x, basket_size):
        if referee_command:
            self.interrupt_robot(referee_command)

        if self.current_state == "find_a_ball":
            action = self.find_a_ball(ball_x, ball_size)
        else:
            action = "Chillin'"

        # print(f"State: {self.current_state}, Ball=(x:{ball_x}|size:{ball_size}, Action: {action}")
        return self.current_state, action

    def find_a_ball(self, ball_x, ball_size):
        """state action"""
        if ball_x == -1:
            hw.move_robot(state="rotation", moving_direction=0, speed_limit=ROBOT_SPEED)
            action = "Cant' see a ball"

        elif ball_x < const.CENTER_RANGE[0]:
            hw.move_robot(state="rotation", moving_direction=-90, speed_limit=ROBOT_SPEED)
            action = "Moving left"

        elif ball_x > const.CENTER_RANGE[-1]:
            hw.move_robot(state="rotation", moving_direction=90, speed_limit=ROBOT_SPEED)
            action = "Moving right"

        elif ball_x in const.CENTER_RANGE:
            if ball_size <= BALL_SIZE_TO_STOP:
                hw.move_robot(state="transition", moving_direction=90, speed_limit=ROBOT_SPEED)
                action = "Moving closer"
            else:
                hw.move_robot(moving_direction=0, speed_limit=0)
                # self.current_state = "ball_found"
                action = "Ball is close"
                
        return action
