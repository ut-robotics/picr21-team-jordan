from pynput import keyboard
from robot_movement import RobotMovement


class ManualController:
    def __init__(self):
        self.robot = RobotMovement()
        self.max_speed = 40
        self.speed_x = 0
        self.speed_y = 0
        self.speed_rot = 0
        self.enable = False
        self.kill = False

    def is_enabled(self):
        return self.enable

    def main(self):
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.nothing)
        listener.start()

    def on_press(self, key):
        try:
            if key.char == "w":
                self.speed_y = self.max_speed if self.speed_y == 0 else 0
            elif key.char == "s":
                self.speed_y = -self.max_speed if self.speed_y == 0 else 0
            elif key.char == "d":
                self.speed_x = self.max_speed if self.speed_x == 0 else 0
            elif key.char == "a":
                self.speed_x = -self.max_speed if self.speed_x == 0 else 0
            elif key.char == "e":
                self.speed_rot = int(-self.max_speed / 3) if self.speed_rot == 0 else 0
            elif key.char == "r":
                self.speed_rot = int(self.max_speed / 3) if self.speed_rot == 0 else 0
            elif key.char == "e":
                self.speed_rot = 0
                self.speed_x = 0
                self.speed_y = 0
            elif key.char == "g":
                self.enable = True if not self.enable else False
        # special key pressed
        except AttributeError:
            pass

    def nothing(self):
        pass


if __name__ == "__main__":
    controller = ManualController()
    controller.main()
