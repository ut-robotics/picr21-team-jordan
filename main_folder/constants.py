# frame size constants
CAM_ID = 4
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

# camera config and image processing constants
WHITE_BALANCE = 3500
EXPOSURE = 50
BLUR = 5
ALPHA_DEPTH = 0.08

# constants for names of windows and string type variables
CONFIG_PATH = "/home/jordan_team/picr21-team-jordan/main_folder/config/"
BALL = "trackbar_values_ball"
BASKET_BLUE = "trackbar_values_basket_blue"
BASKET_ROSE = "trackbar_values_basket_rose"
ORIGINAL_WINDOW = "Original"
MASKED_WINDOW = "Thresh"
MASKED_WINDOW_BASKET = "Basket"
DEPTH_WINDOW = "Depth"
TRACKBAR_WINDOW = "Trackbar"

# detection constants
BLOB_MIN_AREA = 0
BLOB_MAX_AREA = 999_999
MIN_DISTANCE_BETWEEN_BLOBS = 40
MINIMAL_BALL_SIZE_TO_DETECT = 7  # TODO delete mb, this is blob size
MIN_BALL_RADIUS_TO_DETECT = 2
MIN_BALL_RADIUS_TO_STOP = 25
MINIMAL_BASKET_RADIUS_TO_DETECT = 16

# center coordinates and range (that robot claims as center) constants
CENTER_OFFSET_X = 20
CENTER_OFFSET_Y = 20
CENTER_X = int(WIDTH_RESIZED / 2)
# TODO REFACTOR
CENTER_Y = int(HEIGHT_RESIZED / 2)
CENTER_RANGE_X = range(CENTER_X - CENTER_OFFSET_X, CENTER_X + CENTER_OFFSET_X, 1)
CENTER_RANGE_Y = range(CENTER_Y - CENTER_OFFSET_Y, CENTER_Y + CENTER_OFFSET_Y, 1)

# speed calculation constants
MAXIMUM_SPEED = 25
ROT_MULTIPLIER = 10
Y_MULTIPLIER = 5

# TODO sort constants
