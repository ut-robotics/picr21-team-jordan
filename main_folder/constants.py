# frame size constants
WIDTH = 960
HEIGHT = 540
WIDTH_DEPTH = 848
HEIGHT_DEPTH = 480
CROP_Y1 = 170
CROP_Y2 = 350

# constants for names of windows
BASKET = "basket"
BALL = "ball"
ORIGINAL_WINDOW = "Original"
MASKED_WINDOW = "Thresh"
MASKED_WINDOW_BASKET = "Basket"
DEPTH_WINDOW = "Depth"
TRACKBAR_WINDOW = "Trackbar"

# ball detection constants
MINIMAL_BALL_SIZE_TO_DETECT = 7  # TODO delete mb
MIN_BALL_RADIUS = 20

# center coordinates and range (that robot claims as center) constants
CENTER_OFFSET_X = 30
CENTER_OFFSET_Y = 20
CENTER_X = int(WIDTH / 2)
CENTER_Y = int(CROP_Y2 / 2)
CENTER_RANGE_X = range(CENTER_X - CENTER_OFFSET_X, CENTER_X + CENTER_OFFSET_X, 1)
CENTER_RANGE_Y = range(CENTER_Y - CENTER_OFFSET_Y, CENTER_Y + CENTER_OFFSET_Y, 1)

# speed calculation constants
ROT_MULTIPLIER = 10
Y_MULTIPLIER = 5
