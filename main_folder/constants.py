# frame size constants
WIDTH = 960
HEIGHT = 540
WIDTH_DEPTH = 848
HEIGHT_DEPTH = 480
CROP_Y1 = 200
CROP_Y2 = 300
RESIZE_X = 1.2
RESIZE_Y = 2
WIDTH_RESIZED = int(WIDTH * RESIZE_X)
HEIGHT_RESIZED = int(CROP_Y2 * RESIZE_Y)

# constants for names of windows
BASKET_BLUE = "trackbar_values_basket_blue"
BASKET_ROSE = "trackbar_values_basket_rose"
BALL = "trackbar_values_ball"
ORIGINAL_WINDOW = "Original"
MASKED_WINDOW = "Thresh"
MASKED_WINDOW_BASKET = "Basket"
DEPTH_WINDOW = "Depth"
TRACKBAR_WINDOW = "Trackbar"

# ball detection constants
MINIMAL_BALL_SIZE_TO_DETECT = 7  # TODO delete mb
MIN_BALL_RADIUS_TO_DETECT = 0
MIN_BALL_RADIUS_TO_STOP = 25

# center coordinates and range (that robot claims as center) constants
CENTER_OFFSET_X = 20
CENTER_OFFSET_Y = 20
CENTER_X = int(WIDTH_RESIZED / 2)
#TODO REFACTOR
CENTER_Y = int(HEIGHT_RESIZED / 2)
CENTER_RANGE_X = range(CENTER_X - CENTER_OFFSET_X, CENTER_X + CENTER_OFFSET_X, 1)
CENTER_RANGE_Y = range(CENTER_Y - CENTER_OFFSET_Y, CENTER_Y + CENTER_OFFSET_Y, 1)

# speed calculation constants
MAXIMUM_SPEED = 25
ROT_MULTIPLIER = 10
Y_MULTIPLIER = 5
