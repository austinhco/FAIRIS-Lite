# WebotsSim/controllers/lab3/Lab3_Task1.py

import math

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from WebotsSim.libraries.Emoo import Emoo
# Set working directory for maze loading
import os

os.chdir("../..")

# create the Robot instance.
emoo = Emoo()

# reset robot position
# Loads the environment from the maze file
maze_file = 'worlds/mazes/Labs/Lab3/Lab3_Task1.xml'
emoo.load_environment(maze_file)
emoo.move_to_start()

# Set basal readings
emoo.initial_fle = emoo.get_front_left_motor_encoder_reading()
emoo.initial_fre = emoo.get_front_right_motor_encoder_reading()

# Set behavioral parameters
emoo.speed_pref = 10
emoo.angular_speed_pref = 10
emoo.rotational_speed_pref = 2
emoo.linear_precision_pref = 0.005
emoo.angular_precision_pref = 3
emoo.wall_error_precision_pref = 0.05
emoo.braking_distance = 0.5
emoo.braking_velocity = 10
emoo.angular_braking_velocity = 10
emoo.wall_following_speed = 10
emoo.target_angle_width = 10

# Set sensor parameters
emoo.range_width = 10

# Set PID parameters
emoo.pid_kp = 1  # k forward
emoo.pid_ks = 4  # k side

# Set pose estimate
emoo.estimated_x = 0
emoo.estimated_y = 0

# Target properties
emoo.target_size = [1.5, 0.5]  # [Height, Diameter]

while 1:
    # World can be changed on line 18
    # emoo.approach_target(0.3)  # Generic target approach / seek
    emoo.bug_zero("left", 0.3)  # Wall following target approach
    emoo.advance()
