from pynput import keyboard

X_SPEED = 25
Y_SPEED = 25
ROT_SPEED = 10


def on_press(key):
    key = str(key)
    print(key)
    speed_x, speed_y, speed_rot = 0, 0, 0
    if key == 'w':
        speed_x = 25
    if key == "s":
        speed_x = -25
    if key == "d":
        speed_y = 25
    if key == "a":
        speed_y = -25
    if key == "q":
        speed_rot = 10
    if key == "r":
        speed_rot = -10

    print(speed_x, speed_y, speed_rot)


def on_release(key):
    print("{0} released".format(key))
    if key == keyboard.Key.esc:
        return False


with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()
