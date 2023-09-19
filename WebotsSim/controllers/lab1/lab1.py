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
robot.speed_pref = 10
robot.angular_speed_pref = 5
robot.rotational_speed_pref = 1
robot.linear_precision_pref = 0.005
robot.angular_precision_pref = 1
robot.braking_distance = 0.1
robot.braking_velocity = 5
robot.angular_braking_velocity = 2.5

# Set pose estimate
robot.estimated_x = 1.5
robot.estimated_y = -1.5

# Movement Script
waypoints = [[1.5, -1.5], [1.5, 1], [1, 1.5], [-1, 1.5], [-1.5, 1], [-1.5, 0.5], [0, 0.5], [0.5, 0], [0, -0.5],
             [-1, -0.5], [-1.5, -1], [-1, -1.5], [0.5, -1.5]]

for point in waypoints:
    # Calculate distance and angle between current position and next waypoint
    distance_x = point[0] - robot.estimated_x
    distance_y = point[1] - robot.estimated_y
    move_x = 0 if distance_x <= robot.linear_precision_pref else 1
    move_y = 0 if distance_y <= robot.linear_precision_pref else 1
    angle = math.atan2(distance_y, distance_x) * (180 / math.pi)
    print("\n" + str(distance_x) + " " + str(distance_y) + " " + str(angle))
    robot.move_linear(2.5)
    continue

# robot.move_linear(2.5)
# robot.move_arc_angle(90, -0.5)
# robot.move_linear(2)
# robot.move_arc_angle(90, -0.5)
# robot.move_linear(0.5)
# robot.rotate(-90)
# robot.move_linear(1.5)
# robot.move_arc_angle(180, 0.5)
# robot.move_linear(1)
# robot.move_arc_angle(180, -0.5)
# robot.move_linear(1.5)
