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

# Set basal readings
robot.initial_fle = robot.get_front_left_motor_encoder_reading()
robot.initial_fre = robot.get_front_right_motor_encoder_reading()

# Set calculation parameters
robot.speed_pref = 15
robot.angular_speed_pref = 2
robot.rotational_speed_pref = 1
robot.linear_precision_pref = 0.005
robot.angular_precision_pref = 1

# Movement Script
robot.move_linear(2.5)
robot.move_arc_angle(90, -0.5)
