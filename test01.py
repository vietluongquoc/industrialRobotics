# import kinematcRobot
# from ultities import *
# import numpy as np

# robot = kinematcRobot.Robot()

# robot.update_joint_angles([0, 0, 0, 0])
# robot.Tmatrix = robot.forward_kinematics()

# print("True geometry")
# print(robot.geometryKinematics())

# print("DH")
# print(robot.Tmatrix)

# print("MR")
# print(robot.inverse_kinematics( [345.3, 0, 62.8]))
from i2cpy import I2C
import servoPCA9685
import time

motor = servoPCA9685.Servo()
motor.control_servo(3, 90)  # Set servo on channel 0 to 90 degrees
# time.sleep(1)  # Wait for 1 second
# motor.control_servo(3, 90)  # Set servo on channel 0 to 90 degrees  
# time.sleep(1)  # Wait for 1 second
# motor.control_servo(3, 180)  # Set servo on channel 0 to 90 degrees  