import math
import struct
from typing import Optional

import serial
import serial.tools.list_ports


class SerialPortNotFound(Exception):
    def __init__(self):
        super().__init__("Serial port not found")


STM_32_HWID = "USB VID:PID=0483:5740"
WHEEL_ANGLES = [-120, 0, 120]


class RobotMovement:
    def __init__(self):
        serial_port: Optional[str] = None
        ports = serial.tools.list_ports.comports()
        devices = {}

        for port, _, hwid in sorted(ports):
            devices[hwid] = port

        for hwid in devices.keys():
            if STM_32_HWID in hwid:
                serial_port = devices[hwid]
                break

        if serial_port is None:
            raise SerialPortNotFound
        self.ser = serial.Serial(serial_port, 115200)

    def calculate_speed(self, wheel_angle_degrees, moving_direction, speed, rotation_speed):
        return int(round(math.cos((moving_direction + wheel_angle_degrees) * (2 * math.pi / 360)) * speed + rotation_speed))

    def move_robot(self, moving_direction=0, speed=0, rotation_speed=0, thrower_speed=0, failsafe=0):
        speed1, speed2, speed3 = [self.calculate_speed(angle, moving_direction, speed, rotation_speed) for angle in WHEEL_ANGLES]
        send_data = struct.pack("<hhhHBH", speed1, speed2, speed3, thrower_speed, failsafe, 0xAAAA)
        self.ser.write(send_data)

        received_data = self.ser.read(8)
        actual_speed1, actual_speed2, actual_speed3, feedback_delimiter = struct.unpack("<hhhH", received_data)
        # print(f"calucalted/actual | {speed1}/{actual_speed1} | {speed2}/{actual_speed2} | {speed3}/{actual_speed3}")


    def move_robot_XY(self, speed_x=0, speed_y=0, rotation_speed=0, thrower_speed=0):
        moving_direction = (math.atan2(speed_y, speed_x)) * 180 / math.pi
        speed = math.hypot(speed_x, speed_y)  # vector length
        self.move_robot(moving_direction, speed, rotation_speed, thrower_speed)


if __name__ == "__main__":
    robot = RobotMovement()
    for i in range(25000):
        robot.move_robot_XY(0, 0, 0)
