import struct
import serial
import time


ser = serial.Serial("/dev/ttyACM0", 115200)

if not ser.isOpen():
    ser.open()

print(ser.isOpen())


disable_failsafe = 0
speed1 = 10
speed2 = 10
speed3 = 10
thrower_speed = 10

try:
    while True:
        send_data = struct.pack("<hhhHBH", speed1, speed2, speed3, thrower_speed, disable_failsafe, 0xAAAA)
        ser.write(send_data)
        time.sleep(0.5)
        received_data = ser.read(8)
        actual_speed1, actual_speed2, actual_speed3, feedback_delimiter = struct.unpack("<hhhH", received_data)
        print(actual_speed1, "actual_speed1")
        print(actual_speed2, "actual_speed2")
        print(actual_speed3, "actual_speed3")
        print(feedback_delimiter, "feedback_delimiter")
        time.sleep(0.5)
        print("hello")
except KeyboardInterrupt:
    ser.close()
