# from robot_movement import RobotMovement
import tkinter as tk
import threading


def arduino_map(x, in_min=-250, in_max=250, out_min=-40, out_max=40):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


def motion(event):
    x, y = int(event.x - center_x), -int(event.y - center_y)
    move_x, move_y = arduino_map(x), arduino_map(y)
    print(move_x, move_y)


width = 500
height = 500
center_x, center_y = (width / 2, height / 2)
offset = 10
root = tk.Tk()
root.geometry(f"{width}x{height}")


canvas = tk.Canvas(root, width=width, height=height, borderwidth=0, highlightthickness=0, bg="#b5c4ff")
canvas.create_oval((0 + offset, 0 + offset), (width - offset, height - offset), width=3, fill="#96a9f2")
canvas.create_oval((center_x - offset * 5, center_y - offset * 5), (center_x + offset * 5, center_y + offset * 5), width=3, fill="#b5c4ff")
canvas.grid()
canvas.bind("<Motion>", motion)
canvas.bind("<Button-1>", motion)


root.mainloop()
