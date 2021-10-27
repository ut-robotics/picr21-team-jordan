import struct
import serial
import time
import math


def(serial_port="ttyACM0", moving_direction=0, speed_lime=10, thrower_speed=0, failsafe=0):

    dir_serial = "/dev/"+serial_port

    ser = serial.Serial(dir_serial, 115200)
    if not ser.isOpen():
        ser.open()

    print(ser.isOpen())

    # moving_direction = 220  # angel in deg
    # speed_limit = 20
    # thrower_speed = 1200
    # failsafe = 0

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
            # print(actual_speed1, "actual_speed1")
            # print(actual_speed2, "actual_speed2")
            # print(actual_speed3, "actual_speed3")
            # print(feedback_delimiter, "feedback_delimiter")
    except KeyboardInterrupt:
        pass
