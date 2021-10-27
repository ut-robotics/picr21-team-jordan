import struct
import serial
import time
import math


def move_robot(serial_port="ttyACM0", moving_direction=0, speed_lime=10, thrower_speed=0, failsafe=0):
    """Function for moving the robot and controling of the thrower. 
        The function will use serial communication to communicate with with STM32.

    Args:
        serial_port (str, optional): used serial port. Defaults to "ttyACM0".
        moving_direction (int, optional): direction of movement (form 0 to 360). Defaults to 0.
        speed_lime (int, optional): number limits the power of the motors. Defaults to 10.
        thrower_speed (int, optional): PWM value for controlling the thrower (from 0 to ca. 2036). Defaults to 0.
        failsafe (int, optional): booline value to indicate in the robot should repeat the last command. Defaults to 0.
    """

    dir_serial = "/dev/"+serial_port

    ser = serial.Serial(dir_serial, 115200)
    if not ser.isOpen():
        ser.open()

    print(ser.isOpen())

    speed1 = int(math.sin((moving_direction+120)
                 * (2*math.pi/360))*(speed_limit))
    speed2 = int(math.sin((moving_direction)*(2*math.pi/360))*(speed_limit))
    speed3 = int(math.sin((goal_dir_angle-120)*(2*math.pi/360))*(speed_limit))

    try:
        while True:
            send_data = struct.pack("<hhhHBH", speed1, speed2,
                                    speed3, thrower_speed, failsafe, 0xAAAA)
            ser.write(send_data)
            received_data = ser.read(8)
            actual_speed1, actual_speed2, actual_speed3, feedback_delimiter = struct.unpack(
                "<hhhH", received_data)
            # for debugging
            # print(actual_speed1, "actual_speed1")
            # print(actual_speed2, "actual_speed2")
            # print(actual_speed3, "actual_speed3")
            # print(feedback_delimiter, "feedback_delimiter")
    except KeyboardInterrupt:
        pass
