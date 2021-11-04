import math
import struct

import serial

DEFAULT_SERIAL_PORT = "/dev/ttyACM0"

# Inputs
robotSpeedX = 0  # - m/s
robotSpeedY = 0  # - m/s

robotAngularVelocity = 0.01
wheelLinearVelocity = 0.01  # [m/s]

# motor variables
gearboxReductionRatio = 18.75
encoderEdgesPerMotorRevolution = 64
wheelRadius = 0.035  # $[m]
pidControlFrequency = 60  # [Hz]
wheelLinearVelocity = 2  # [m/s]
wheelDistanceFromCenter = 0.145 # [m]
wheelAngle_1 = (120*math.pi)/360
wheelAngle_2 = (0*math.pi)/360
wheelAngle_3 = (-120*math.pi)/360


def cal_robotSpeed(robotSpeedX, robotSpeedY):
    robotSpeed = math.sqrt(robotSpeedX**2 + robotSpeedY**2)
    print(robotSpeed, "robotSpeed")
    return robotSpeed


def cal_robotDirectionAngle(robotSpeedX, robotSpeedY):
    robotDirectionAngle = math.atan2(robotSpeedY, robotSpeedX)
    return robotDirectionAngle


def cal_speed(robotSpeed, robotDirectionAngle, wheelAngle, wheelDistanceFromCenter, robotAngularVelocity):
    wheelLinearVelocity = robotSpeed * math.cos(robotDirectionAngle - wheelAngle) + wheelDistanceFromCenter * robotAngularVelocity
    return wheelLinearVelocity


def cal_seed_to_board(wheelLinearVelocity, gearboxReductionRatio, encoderEdgesPerMotorRevolution, wheelRadius, pidControlFrequency):
    wheelSpeedToMainboardUnits = gearboxReductionRatio * \
        encoderEdgesPerMotorRevolution / \
        (2 * math.pi * wheelRadius * pidControlFrequency)
    wheelAngularSpeedMainboardUnits = wheelLinearVelocity * wheelSpeedToMainboardUnits
    return wheelAngularSpeedMainboardUnits


def move_robot(robotSpeedX, robotSpeedY, speed_limit=0.5, thrower_speed=0, failsafe=0, state="translation"):

    ser = serial.Serial(DEFAULT_SERIAL_PORT, 115200)
    if not ser.isOpen():
        ser.open()

    if state == "translation":
        print(robotSpeedX)
        print(robotSpeedY)
        print(speed_limit)
        robotSpeed = cal_robotSpeed(robotSpeedX, robotSpeedY) * speed_limit
        robotDirectionAngle = cal_robotDirectionAngle(robotSpeedX, robotSpeedY)
        
        print(robotSpeed)
        print(robotDirectionAngle)

        wheelLinearVelocity_1 = cal_speed(
            robotSpeed, robotDirectionAngle, wheelAngle_1, wheelDistanceFromCenter, robotAngularVelocity)
        speed1 = cal_seed_to_board(wheelLinearVelocity, gearboxReductionRatio,
                                   encoderEdgesPerMotorRevolution, wheelRadius, pidControlFrequency)

        wheelLinearVelocity_2 = cal_speed(
            robotSpeed, robotDirectionAngle, wheelAngle_2, wheelDistanceFromCenter, robotAngularVelocity)
        speed2 = cal_seed_to_board(wheelLinearVelocity, gearboxReductionRatio,
                                   encoderEdgesPerMotorRevolution, wheelRadius, pidControlFrequency)

        wheelLinearVelocity_3 = cal_speed(
            robotSpeed, robotDirectionAngle, wheelAngle_3, wheelDistanceFromCenter, robotAngularVelocity)
        speed3 = cal_seed_to_board(wheelLinearVelocity, gearboxReductionRatio,
                                   encoderEdgesPerMotorRevolution, wheelRadius, pidControlFrequency)

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
        print(speed1)
        send_data = struct.pack("<hhhHBH", int(speed1), int(speed2),
                                int(speed3), int(thrower_speed), failsafe, 0xAAAA)
        ser.write(send_data)
        received_data = ser.read(8)
        actual_speed1, actual_speed2, actual_speed3, feedback_delimiter = struct.unpack(
            "<hhhH", received_data)
        # for debugging
        print(
            f"{speed1}/{actual_speed1} | {speed2}/{actual_speed2} | {speed3}/{actual_speed3}")
        # print(feedback_delimiter, "feedback_delimiter")
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    move_robot(robotSpeedX=0.0, robotSpeedY=0.0, speed_limit=0, state= "translation")


