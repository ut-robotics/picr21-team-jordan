import math
import struct

import serial

DEFAULT_SERIAL_PORT = "/dev/ttyACM0"


def calculate_speed(degrees, moving_direction, speed_limit):
    return int(math.sin((moving_direction + degrees) * (2 * math.pi / 360)) * (speed_limit))


def move_robot(state="transition", moving_direction=0, speed_limit=10, thrower_speed=0, failsafe=0):
    """
        Moving the robot and controling of the thrower.
        Used serial communication to communicate with with STM32.
    Args:
        moving_direction: 0-360 deg
        speed_limit: (~100 max)
        thrower_speed: PWM value for controlling the thrower (from 0 to ca. 2036)
        failsafe: BOOL value to indicate in the robot should repeat the last command
    """

    ser = serial.Serial(DEFAULT_SERIAL_PORT, 115200)
    if not ser.isOpen():
        ser.open()

    if state == "transition":
        speed1 = calculate_speed(120, moving_direction, speed_limit)
        speed2 = calculate_speed(0, moving_direction, speed_limit)
        speed3 = calculate_speed(-120, moving_direction, speed_limit)
        """
        !TODO delete this after tested on robot
        speed1 = int(math.sin((moving_direction + 120) * (2 * math.pi / 360)) * (speed_limit))
        speed2 = int(math.sin((moving_direction + 0) * (2 * math.pi / 360)) * (speed_limit))
        speed3 = int(math.sin((moving_direction - 120) * (2 * math.pi / 360)) * (speed_limit))
        """
    elif state == "rotation":
        speed1 = speed_limit
        speed2 = speed_limit
        speed3 = speed_limit
    else:
        raise Exception("Invalid state arrgument.")

    try:
        send_data = struct.pack("<hhhHBH", speed1, speed2, speed3, thrower_speed, failsafe, 0xAAAA)
        ser.write(send_data)
        received_data = ser.read(8)
        actual_speed1, actual_speed2, actual_speed3, feedback_delimiter = struct.unpack("<hhhH", received_data)
        # for debugging
        print(f"{speed1}/{actual_speed1} | {speed2}/{actual_speed2} | {speed3}{actual_speed3}")
        # print(feedback_delimiter, "feedback_delimiter")
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    move_robot("ttyACM0", moving_direction=0, speed_limit=0)
