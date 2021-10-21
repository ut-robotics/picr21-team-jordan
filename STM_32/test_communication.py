import struct
import serial
import time
import math


ser = serial.Serial("/dev/ttyACM0", 115200)

if not ser.isOpen():
    ser.open()

print(ser.isOpen())

goal_dir_angle = -90
speed_limit = 30

disable_failsafe = 0
speed1 = int(math.sin((-goal_dir_angle+60)*(2*math.pi/360))*(speed_limit))
speed2 = int(math.sin((-goal_dir_angle+180)*(2*math.pi/360))*(speed_limit))
speed3 = int(math.sin((-goal_dir_angle+240)*(2*math.pi/360))*(speed_limit))

# print(math.sin(goal_dir_angle+60*(2*math.pi/360)))
print(speed1)
print(speed2)
print(speed3)

# speed1 = 0
# speed2 = 0
# speed3 = 0
thrower_speed = 0

try:
    while True:
        send_data = struct.pack("<hhhHBH", speed1, speed2,
                                speed3, thrower_speed, disable_failsafe, 0xAAAA)
        ser.write(send_data)
        received_data = ser.read(8)
        actual_speed1, actual_speed2, actual_speed3, feedback_delimiter = struct.unpack(
            "<hhhH", received_data)
        print(actual_speed1, "actual_speed1")
        print(actual_speed2, "actual_speed2")
        print(actual_speed3, "actual_speed3")
        print(feedback_delimiter, "feedback_delimiter")
        # print("hello")
except KeyboardInterrupt:
    ser.close()
