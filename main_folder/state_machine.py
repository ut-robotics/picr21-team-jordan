class StateMachine(object):
    def __init__(self):
        self.states = ["find_a_ball", "grab_a_ball", "find_a_basket", "throw"]
        self.current_state_index = 0

    def next_state(self):
        self.current_state_index += 1
        if self.current_state_index == len(self.states):
            self.current_state_index = 0

    def run_current_state(self, BALL_X, BALL_SIZE, BASKET_X, BASKET_SIZE):
        # TODO actual movement, state updating
        print(f"State: {self.states[self.current_state_index]}, Ball=(x:{BALL_X}|size:{BALL_SIZE})")


if __name__ == "__main__":
    # example
    robot = StateMachine()
    for i in range(5):
        robot.run_current_state(320, 100, -1, -1)
        robot.next_state()
