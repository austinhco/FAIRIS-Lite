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
maze_file = 'worlds/mazes/Labs/Lab1/Lab1.xml'
robot.load_environment(maze_file)
robot.move_to_start()

# Set basal readings
robot.initial_fle = robot.get_front_left_motor_encoder_reading()
robot.initial_fre = robot.get_front_right_motor_encoder_reading()

# Set calculation parameters
robot.speed_pref = 20
robot.angular_speed_pref = 20
robot.rotational_speed_pref = 2
robot.linear_precision_pref = 0.005
robot.angular_precision_pref = 1
robot.braking_distance = 0.1
robot.braking_velocity = 10
robot.angular_braking_velocity = 10

# Set pose estimate
robot.estimated_x = 1.5
robot.estimated_y = -1.5

# Movement Script
waypoints = [[1.5, -1.5], [1.5, 1], [1, 1.5], [-1, 1.5], [-1.5, 1], [-1.5, 0.5], [0, 0.5], [0.5, 0], [0, -0.5],
             [-1, -0.5], [-1.5, -1], [-1, -1.5], [0.5, -1.5]]

# Speak up!
robot.noisy = True

for point in waypoints:
    print("\n")
    # Calculate distance and angle between current position and next waypoint
    distance_x = point[0] - robot.estimated_x
    distance_y = point[1] - robot.estimated_y
    distance = math.sqrt(math.pow(distance_x, 2) + math.pow(distance_y, 2))
    if not distance:
        continue
    move_x = False if math.fabs(distance_x) <= robot.linear_precision_pref*40 else True
    move_y = False if math.fabs(distance_y) <= robot.linear_precision_pref*40 else True
    angle = math.atan2(distance_y, distance_x) * (180 / math.pi)
    angle_dif = angle - robot.get_compass_reading()
    while angle_dif > 180:
        angle_dif -= 360
    while angle_dif < -180:
        angle_dif += 360
    print("X Distance: " + str(round(distance_x, 2)) + "m")
    print("Y Distance: " + str(round(distance_y, 2)) + "m")
    print("  Distance: " + str(round(distance, 2)) + "m")
    print("     Angle: " + str(round(angle)) + "°")
    # Attempt to rectify angle if performing a linear motion, then move
    if not (move_x and move_y):
        robot.rotate(-angle_dif)
        robot.move_linear(distance)
    else:
        # Movement is arching: additional calculations required
        radius = math.sqrt(math.pow(distance, 2)/2)
        # If destination is relative right, clockwise. If left, counter.
        if angle_dif > 0:
            robot.move_arc_angle(angle_dif*2, -radius)
        elif angle_dif < 0:
            robot.move_arc_angle(angle_dif*2, radius)
