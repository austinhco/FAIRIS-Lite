# WebotsSim/controllers/lab1/Lab1_Task1.py

import math

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from WebotsSim.libraries.Emoo import MyRobot
# Set working directory for maze loading
import os

os.chdir("../..")

# create the Robot instance.
robot = MyRobot()

# reset robot position
# Loads the environment from the maze file
maze_file = 'worlds/mazes/Labs/Lab2/Lab2_1.xml'
robot.load_environment(maze_file)
robot.move_to_start()

# Set basal readings
robot.initial_fle = robot.get_front_left_motor_encoder_reading()
robot.initial_fre = robot.get_front_right_motor_encoder_reading()

# Set behavioral parameters
robot.speed_pref = 20
robot.angular_speed_pref = 20
robot.rotational_speed_pref = 2
robot.linear_precision_pref = 0.01
robot.angular_precision_pref = 1
robot.wall_error_precision_pref = 0.05
robot.braking_distance = 0.5
robot.braking_velocity = 10
robot.angular_braking_velocity = 10
robot.wall_avoidance_aggression = 1.2

# Set sensor parameters
robot.range_width = 5

# Set PID parameters
robot.pid_k = 1  # kp

# Set pose estimate
robot.estimated_x = 0
robot.estimated_y = -1.5

robot.move_arc_distance(0.1, -0.5)
while 1:
    robot.move_through(0.5, 0.3)
    robot.advance()
