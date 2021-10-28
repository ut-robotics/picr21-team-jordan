import move_function
import constants as const

BALL_SIZE_TO_STOP = 90

class StateMachine():
    """
    This class manipulates robot depends of the ball and the basket coords.
    Manipulating is mechanical (turn left, right, etc.)
    """
    def __init__(self):
        self.states = ["find_a_ball", "grab_a_ball", "find_a_basket", "throw"]
        self.current_state_index = 0
        
    def next_state(self):
        self.current_state_index += 1
        if self.current_state_index == len(self.states):
            self.current_state_index = 0

    def run_current_state(self, ball_x, ball_size, basket_x, basket_size):
        # print(f"State: {self.states[self.current_state_index]}, Ball=(x:{ball_x}|size:{BALL_SIZE})")
        if ball_x < const.CENTER_RANGE[0]:
            # robot_movement.turn_right()
            action = "Turning left"
        elif ball_x > const.CENTER_RANGE[-1]:
            action = "Turning right"
        else:
            if ball_size <= BALL_SIZE_TO_STOP:
                action = "moving forward"
            else:
                action= "stop"
        return action
        #TODO add actual states and refactor this stairs mess 