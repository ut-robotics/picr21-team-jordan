import move_function

class StateMachine():
    """
    This class manipulates robot depends of the ball and the basket coords.
    Manipulating is mechanical (turn left, right, etc.)
    """
    def __init__(self):
        self.BALL_SIZE_TO_STOP = 90
        self.CENTER_RANGE = range(1)
        self.CENTER_OFFSET = 0
        self.states = ["find_a_ball", "grab_a_ball", "find_a_basket", "throw"]
        self.current_state_index = 0
        
    def update_center_range(self, center_range):
        self.CENTER_RANGE = center_range

    def next_state(self):
        self.current_state_index += 1
        if self.current_state_index == len(self.states):
            self.current_state_index = 0

    def run_current_state(self, BALL_X, BALL_SIZE, BASKET_X, BASKET_SIZE):
        # print(f"State: {self.states[self.current_state_index]}, Ball=(x:{BALL_X}|size:{BALL_SIZE})")
        if BALL_X < self.CENTER_RANGE[0]:
            # robot_movement.turn_right()
            action = "Turning left"
        elif BALL_X > self.CENTER_RANGE[-1]:
            action = "Turning right"
        else:
            if BALL_SIZE <= self.BALL_SIZE_TO_STOP:
                action = "moving forward"
            else:
                action= "stop"
        return action
        #TODO add actual states and refactor this stairs mess 
        


if __name__ == "__main__":
    # example
    robot = StateMachine()
    for i in range(5):
        robot.run_current_state(320, 100, -1, -1)
        robot.next_state()
