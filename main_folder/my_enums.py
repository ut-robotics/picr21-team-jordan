from enum import Enum

"""im not sure if use this classes or not. TODO mb delete"""

class Frame():
    pass

class State(Enum):
    INITIAL = 0
    FIND_BALL = 1
    GET_TO_BALL = 2
    CENTER_BASKET_AND_BALL = 3


class Color(Enum):
    GREEN = (0, 100, 20)
    BLACK = (0, 0, 0)
    BLUE = (0, 0, 150)
    RED = (100, 0, 0)


class Object(Enum):
    BALL = "trackbar_values_ball"
    BASKET_BLUE = "trackbar_values_basket_blue"
    BASKET_ROSE = "trackbar_values_basket_rose"


class Position(Enum):
    LEFT = "left"
    RIGHT = "right"


class Window(Enum):
    ORIGINAL = "Original"
    MASKED = "Thresh"
    MASKED_BASKET = "Basket"
    DEPTH = "Depth"
    TRACKBAR = "Trackbar"


