import kinematcRobot
from ultities import *
import numpy as np

robot = kinematcRobot.Robot()

robot.update_joint_angles([0, 0, 0, 0])
robot.Tmatrix = robot.forward_kinematics()

print("True geometry")
print(robot.geometryKinematics())

print("DH")
print(robot.Tmatrix)

print("MR")
print(robot.inverse_kinematics( [345.3, 0, 62.8]))