import math
import struct
from typing import Optional

import serial
import serial.tools.list_ports

STM_32_HWID = "USB VID:PID=0483:5740"
WHEEL_ANGLES = [math.radians(angle) for angle in [120, 0, -120]]


class SerialPortNotFound(Exception):
    def __init__(self):
        super().__init__("Serial port not found")

class RobotMovement:
    """
    This class controls motors and thrower by sending encoded speed values to the hardware by USB.
    Uses Omni-Motion to calculate speed.
    """
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

    def calculate_speed(self, wheel_angle, moving_direction, speed, rotation_speed):
        return int(round(math.cos(moving_direction + wheel_angle) * speed + rotation_speed))

    def move_robot(self, moving_direction=0, speed=0, rotation_speed=0, thrower_speed=0):
        speed1, speed2, speed3 = [self.calculate_speed(angle, moving_direction, speed, rotation_speed) for angle in WHEEL_ANGLES]
        send_data = struct.pack("<hhhHH", speed1, speed2, speed3, thrower_speed, 0xAAAA)

        self.ser.write(send_data)

    def arduino_map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def move_robot_XY(self, speed_x=0, speed_y=0, rotation_speed=0, thrower_speed=0):
        thrower_speed = int((((thrower_speed - 0) * (9000 - 3000)) / (5000 - 0)) + 3000)
        moving_direction = math.atan2(speed_y, speed_x)
        speed = math.hypot(speed_x, speed_y)
        self.move_robot(moving_direction, speed, rotation_speed, thrower_speed)
        
if __name__ == "__main__":
    robot = RobotMovement() 
    robot.move_robot_XY(0, 0, 0, 0)
