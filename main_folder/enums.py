from enum import Enum

import numpy as np


class Color(Enum):
    OTHER = 0, np.array([0, 0, 0], dtype=np.uint8)
    GREEN = 1, np.array([0, 255, 0], dtype=np.uint8)
    MAGENTA = 2, np.array([255, 0, 255], dtype=np.uint8)
    BLUE = 3, np.array([255, 0, 0], dtype=np.uint8)
    ORANGE = 4, np.array([0, 127, 255], dtype=np.uint8)
    WHITE = 5, np.array([255, 255, 255], dtype=np.uint8)
    BLACK = 6, np.array([64, 64, 64], dtype=np.uint8)

    def __new__(cls, value, color):
        enum = object.__new__(cls)
        enum._value_ = value
        enum.color = color
        return enum

    def __int__(self):
        return self.value


class State(Enum):
    INITIAL = 0
    FIND_BALL = 1
    GET_TO_BALL = 2
    FIND_A_BASKET = 3
    CENTER_A_BASKET = 4


class ColorRGB:
    GREEN = (0, 100, 20)
    BLACK = (0, 0, 0)
    BLUE = (0, 0, 150)
    RED = (100, 0, 0)


class GameObject:
    BALL = "trackbar_values_ball"
    BASKET_BLUE = "trackbar_values_basket_blue"
    BASKET_ROSE = "trackbar_values_basket_rose"


class Position(Enum):
    LEFT = "left"
    RIGHT = "right"


class Window:
    ORIGINAL = "Original"
    MASKED = "Thresh"
    MASKED_BASKET = "Basket"
    DEPTH = "Depth"
    TRACKBAR = "Trackbar"
