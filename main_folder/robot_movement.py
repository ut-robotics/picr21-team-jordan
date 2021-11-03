import math
import struct
from typing import Optional

import serial
import serial.tools.list_ports


class SerialPortNotFound(Exception):
    def __init__(self):
        super().__init__("Serial port not found")


STM_32_HWID = "USB VID:PID=0483:5740"
WHEEL_ANGLES = [120, 0, -120]


class RobotMovement:
    def __init__(self):
        serial_port: Optional[str] = None
        ports = serial.tools.list_ports.comports()
        devices = {}

        for port, _, hwid in sorted(ports):
            devices[hwid] = port

        for hwid in devices.keys():
            if STM_32_HWID in hwid:
                serial_port: str = devices[hwid]
                break
        
        if serial_port is None:
            raise SerialPortNotFound 
        self.ser = serial.Serial(serial_port, 115200)

    def calculate_speed(self, degrees, moving_direction, speed_limit, state):
        return int(math.sin((moving_direction + degrees) * (2 * math.pi / 360)) * (speed_limit)) if state == "transition" else speed_limit

    def move_robot(self, state="transition", moving_direction=0, speed_limit=10, thrower_speed=0, failsafe=0):
        if not self.ser.isOpen():
            self.ser.open()

        speed1, speed2, speed3 = [self.calculate_speed(angle, moving_direction, speed_limit, state) for angle in WHEEL_ANGLES]

        try:
            send_data = struct.pack("<hhhHBH", speed1, speed2, speed3, thrower_speed, failsafe, 0xAAAA)
            self.ser.write(send_data)
            received_data = self.ser.read(8)
            actual_speed1, actual_speed2, actual_speed3, feedback_delimiter = struct.unpack("<hhhH", received_data)
            # print(f"{speed1}/{actual_speed1} | {speed2}/{actual_speed2} | {speed3}{actual_speed3}")
        except KeyboardInterrupt:
            pass
