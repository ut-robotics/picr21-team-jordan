import pickle as pickle

import cv2
import numpy as np

import camera
# from Color import *
from enums import Color, Window


def nothing(x):
    pass


CONFIG_PATH = "/home/jordan_team/picr21-team-jordan/main_folder/colors/colors.pkl"

cv2.namedWindow(Window.TRACKBAR)
cv2.namedWindow(Window.ORIGINAL)
cv2.namedWindow(Window.MASKED)
cv2.moveWindow(Window.MASKED, 400, 0)

try:
    with open(CONFIG_PATH, "rb") as fh:
        colors_lookup = pickle.load(fh)
except:
    colors_lookup = np.zeros(0x1000000, dtype=np.uint8)

cap = camera.RealsenseCamera()

cv2.createTrackbar("brush_size", Window.TRACKBAR, 3, 10, nothing)
cv2.createTrackbar("noise", Window.TRACKBAR, 1, 5, nothing)

mouse_x = 0
mouse_y = 0
brush_size = 1
noise = 1
p = 0

keyDict = {
    ord("g"): Color.GREEN,
    ord("m"): Color.MAGENTA,
    ord("b"): Color.BLUE,
    ord("f"): Color.ORANGE,
    ord("w"): Color.WHITE,
    ord("d"): Color.BLACK,
    ord("o"): Color.OTHER,
}


def change_color(noise, brush_size, mouse_x, mouse_y):
    ob = rgb[max(0, mouse_y - brush_size) : min(cap.rgb_height, mouse_y + brush_size + 1), max(0, mouse_x - brush_size) : min(cap.rgb_width, mouse_x + brush_size + 1), :].reshape((-1, 3)).astype("int32")
    noises = range(-noise, noise + 1)
    for r in noises:
        for g in noises:
            for b in noises:
                colors_lookup[((ob[:, 0] + r) + (ob[:, 1] + g) * 0x100 + (ob[:, 2] + b) * 0x10000).clip(0, 0xFFFFFF)] = p


# mouse callback function
def choose_color(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_x = x
        mouse_y = y
        brush_size = cv2.getTrackbarPos("brush_size", Window.TRACKBAR)
        noise = cv2.getTrackbarPos("noise", Window.TRACKBAR)
        change_color(noise, brush_size, mouse_x, mouse_y)


cv2.namedWindow(Window.ORIGINAL)
cv2.setMouseCallback(Window.ORIGINAL, choose_color)
cv2.setMouseCallback(Window.MASKED, choose_color)

print("Quit: 'q', Save 's', Erase selected color 'e'")
print("Balls 'g', Magenta basket='m', Blue basket='b', Field='f', White='w', Black='d', Other='o'")

cap.open()

while True:
    rgb, depth = cap.get_frames()
    fragmented = colors_lookup[rgb[:, :, 0] + rgb[:, :, 1] * 0x100 + rgb[:, :, 2] * 0x10000]
    frame = np.zeros((cap.rgb_height, cap.rgb_width, 3), dtype=np.uint8)
    for color in Color:
        frame[fragmented == int(color)] = color.color

    cv2.imshow(Window.ORIGINAL, rgb)
    cv2.imshow(Window.MASKED, frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    elif key in keyDict:
        col = keyDict[key]
        print(col)
        p = int(col)
    elif key == ord("s"):
        with open(CONFIG_PATH, "wb") as fh:
            pickle.dump(colors_lookup, fh, -1)
        print("saved")
    elif key == ord("e"):
        print("erased")
        colors_lookup[colors_lookup == p] = 0

cap.close()
cv2.destroyAllWindows()
