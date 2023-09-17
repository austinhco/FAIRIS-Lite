"""lab1 controller."""
import math

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from WebotsSim.libraries.MyRobot import MyRobot
# Set working directory for maze loading
import os

os.chdir("../..")

# create the Robot instance.
robot = MyRobot()

# reset robot position
# Loads the environment from the maze file
maze_file = 'worlds/mazes/Labs/Lab1/Lab1.xml'
robot.load_environment(maze_file)
robot.move_to_start()

# Movement Script
robot.move_linear(4)
