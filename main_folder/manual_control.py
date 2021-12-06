from pynput import keyboard
from robot_movement import RobotMovement


class ManualController:
    def __init__(self):
        self.robot = RobotMovement()
        self.max_speed = 40
        self.speed_x = 0
        self.speed_y = 0
        self.speed_rot = 0
        self.kill = False

    def main(self):
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

        while True:
            if self.kill:
                break
            # print(self.speed_x, self.speed_y, self.speed_rot)
            # self.robot.move_robot_XY(self.speed_x, self.speed_y, self.speed_rot)

    def on_press(self, key):
        try:
            if key.char == "w":
                self.speed_y = self.max_speed           
                print("pressed")
            elif key.char == "s":
                self.speed_y = -self.max_speed
            elif key.char == "d":
                self.speed_x = self.max_speed
            elif key.char == "a":
                self.speed_x = -self.max_speed
            elif key.char == "r":
                self.speed_rot = int(self.max_speed / 4)
            elif key.char == "q":
                self.speed_rot = int(-self.max_speed / 4)
        # special key pressed
        except AttributeError:
            pass

    def on_release(self, key):
        if key == keyboard.Key.esc:
            self.kill = True
            return False
        try:
            if key.char == "w" or key.char == "s":
                print("released")
                self.speed_y = 0
            elif key.char == "a" or key.char == "d":
                self.speed_x = 0
            elif key.char == "q" or key.char == "r":
                self.speed_rot = 0
        except AttributeError:
            pass


if __name__ == "__main__":
    controller = ManualController()
    controller.main()
