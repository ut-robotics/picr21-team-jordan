from pynput import keyboard


class ManualController:
    """
    Class that listens to key input and changes robot speed values.
    """
    def __init__(self):
        self.thrower_max_speed = 3000
        self.max_speed = 40
        self.speed_x = 0
        self.speed_y = 0
        self.speed_rot = 0
        self.speed_throw = 0
        self.enable = False
        self.kill = False

    def main(self):
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.nothing)
        listener.start()

    def on_press(self, key):
        if key.char == "g":
            self.enable = True if self.enable == False else False
            print(f"Running manual control = {self.enable}")

        if self.enable:
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
                elif key.char == "q":
                    self.speed_rot = int(self.max_speed / 3) if self.speed_rot == 0 else 0
                elif key.char == "t":
                    self.speed_rot = 0
                    self.speed_x = 0
                    self.speed_y = 0
                elif key.char == "m":
                    self.speed_throw += 10 if self.speed_throw != self.thrower_max_speed else self.thrower_max_speed
                    print(self.speed_throw)
                elif key.char == "n":
                    self.speed_throw -= 10 if self.speed_throw != 0 else 0
                    print(self.speed_throw)
            # special key pressed
            except AttributeError:
                pass

    def nothing(self, key):
        pass
