# WebotsSim/controllers/lab4/Lab4_Task1.py

import math

import numpy

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
maze_file = 'worlds/mazes/Labs/Lab4/Lab4_Task2_3.xml'
emoo.load_environment(maze_file)
emoo.move_to_start()

# Set basal readings
emoo.initial_fle = emoo.get_front_left_motor_encoder_reading()
emoo.initial_fre = emoo.get_front_right_motor_encoder_reading()

# Status printing
emoo.noisy = False
emoo.pose_always = False
emoo.grid_always = False

# Set behavioral parameters
emoo.speed_pref = 10
emoo.angular_speed_pref = 10
emoo.rotational_speed_pref = 0.8
emoo.linear_precision_pref = 0.005
emoo.angular_precision_pref = 1
emoo.wall_error_precision_pref = 0.05
emoo.braking_distance = 0.5
emoo.braking_velocity = 10
emoo.angular_braking_velocity = 10
emoo.wall_following_speed = 10
emoo.target_angle_width = 10
emoo.measure_steps = 1

# Set sensor parameters
emoo.range_width = 5

# Set PID parameters
emoo.pid_kp = 1  # k forward
emoo.pid_ks = 4  # k side

# Set pose estimate
emoo.estimated_x = 0
emoo.estimated_y = 0

# Target properties
emoo.target_size = [1.5, 0.5]  # [Height, Diameter]

# Localization info NOTE: Cells are organized such that UP/NORTH is NEGATIVE
emoo.grid_dims = [42, 42]
emoo.cell_len = 0.2
emoo.true_map_dims = [21, 21]
# emoo.landmarks = [[[1, 1, 0], [-2, 2]],
#                   [[0, 1, 0], [-2, -2]],
#                   [[1, 0, 0], [2, 2]],
#                   [[0, 0, 1], [2, -2]]]
# Task 2 Map 1
# emoo.cell_walls = [['WN', 'SN', 'SN', 'NE'],
#                    ['SW', 'SN', 'NE', 'WE'],
#                    ['WN', 'SN', 'SE', 'WE'],
#                    ['SW', 'SN', 'SN', 'SE']]
# Task 2 Map 2
# emoo.cell_walls = [['WN', 'SN', 'SN', 'NE'],
#                    ['SW', 'N', 'NE', 'WE'],
#                    ['WNE', 'SW', 'SE', 'WE'],
#                    ['SW', 'SN', 'SN', 'SE']]
# Task 2 Map 3
# emoo.cell_walls = [['SNW', 'SN', 'N', 'NE'],
#                    ['WN', 'NE', 'WE', 'WE'],
#                    ['WE', 'SW', 'SE', 'WE'],
#                    ['SW', 'SN', 'SN', 'SE']]
emoo.cell_walls = numpy.transpose(numpy.array(emoo.cell_walls))
# Initialize positional probabilities
for j in range(emoo.grid_dims[1]):
    position_row = []
    occupancy_row = []
    for i in range(emoo.grid_dims[0]):
        position_row.append(0.5)
        occupancy_row.append(0.5)
    emoo.cell_probs.append(position_row)
    emoo.occupancy_matrix.append(occupancy_row)
# starting_cell = emoo.coord_to_cell(emoo.starting_position.x+2, emoo.starting_position.y-2)
# emoo.assert_probs_position(starting_cell[0], starting_cell[1])

emoo.advance()
# World can be changed on line 18
emoo.navigate_occupancy()
emoo.print_grid()
emoo.compress_occupancy_matrix()
emoo.print_grid()
# Advance time
emoo.advance()
