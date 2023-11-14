# WebotsSim/libraries/Emoo.py

import math
import statistics
import sys

import numpy

from WebotsSim.libraries.RobotLib.RosBot import RosBot


class Emoo(RosBot):

    def __init__(self):
        RosBot.__init__(self)

    # Properties
    camera_resolution = [633, 633]  # X, Y. Y not tested - assumed square
    camera_fov = [62, 62]  # X, Y. Y not tested - assumed square

    # Print velocities and times before moving
    noisy = False
    # Print pose at every timestep
    pose_always = True
    # print grid at every timestep
    grid_always = False

    # Behavioral Preference Parameters
    speed_pref = 5  # radians per second per wheel
    angular_speed_pref = 2  # radians per second relative to ICC
    rotational_speed_pref = 1  # radians per second per wheel
    linear_precision_pref = 0.001  # meters
    angular_precision_pref = 1  # degrees
    wall_error_precision_pref = 0.1  # meters
    braking_distance = 0.25  # meters
    braking_velocity = 1  # radians per second per wheel
    angular_braking_velocity = 1  # radians per second per wheel
    wall_following_speed = 1  # error cap for wall following. higher values -> less aggression
    target_angle_width = 10  # range to be considered facing target

    # Sensor Preference Parameters
    range_width = 30  # Width of lidar readings in degrees (averages values)

    # PID Preference Parameters
    pid_kp = 1
    pid_ks = 1

    # State string
    state = "STOPPED"

    # Last wall readings
    last_left_wall_distance = 0
    last_right_wall_distance = 0

    # Basal Sensor Readings
    initial_fle = 0
    initial_fre = 0

    # Position estimates
    estimated_x = 0
    estimated_y = 0
    cell_x = 0
    cell_y = 0

    # Last encoder readings
    last_fle = 0
    last_fre = 0

    # Camera target information
    target_size = [0, 0]

    # Map and landmark information
    grid_dims = [1, 1]  # Size of grid in cells
    cell_len = 1  # Size of cell in m (cells = cell_len x cell_len squares)
    landmarks = []  # Landmarks: List of landmarks as [[r,g,b], [x, y]]
    cell_walls = []  # Wall characteristics for all walls
    cell_probs = []  # Probabilities (that emoo is in it) for each cell
    visited = set([])  # list of visited cells
    occupied = set([])  # list of obstacle cells
    previous_moves = []  # list of prior cell movements

    # Override stop function to add state update
    def stop(self):
        self.set_state("STOPPED")
        self.set_left_motors_velocity(0)
        self.set_right_motors_velocity(0)

    # Set state of bot
    def set_state(self, state):
        self.state = state
        if "TURN" not in state:
            self.reset_last_wall_readings()

    # Reset last wall readings
    def reset_last_wall_readings(self):
        self.last_right_wall_distance = 0
        self.last_left_wall_distance = 0

    # Update position estimate
    def update_estimates(self):
        if "ROTATE" in self.state:
            return
        heading = self.get_compass_reading() * (math.pi / 180)
        distance = self.calculate_distance(statistics.mean([self.last_fle, self.last_fre]))
        self.estimated_x += distance * math.cos(heading)
        self.estimated_y += distance * math.sin(heading)
        self.last_fre = self.relative_fre()
        self.last_fle = self.relative_fle()
        return

    # Calculate distance travelled since input encoder start point
    def calculate_distance(self, initial_average_encoder_reading):
        return (statistics.mean(
            [self.relative_fre(), self.relative_fle()]) - initial_average_encoder_reading) * self.wheel_radius

    def print_decision(self, distance, velocity_l, velocity_r):
        speed = statistics.mean([velocity_l, velocity_r])
        print("     Speed: " + str(round(speed * self.wheel_radius, 2)) + "m/s")
        print("L Velocity: " + str(round(velocity_l * self.wheel_radius, 2)) + "m/s")
        print("R Velocity: " + str(round(velocity_r * self.wheel_radius, 2)) + "m/s")
        print("       ETA: " + str(round((distance / (speed * self.wheel_radius)) + (
                self.braking_distance / (self.braking_velocity * self.wheel_radius)), 2)) + "s")

    # Move until a set distance has been travelled. If arc is set, brake as such using movement sign
    def move_until(self, initial_average_encoder_reading, distance, arc=False, sign=0, velocity_outer=0,
                   velocity_inner=0):
        while self.calculate_distance(initial_average_encoder_reading) <= distance - self.linear_precision_pref:
            if arc:
                brake_mult = self.angular_braking_velocity / statistics.mean([velocity_outer, velocity_inner])
                if sign:
                    self.set_left_motors_velocity(velocity_outer * brake_mult)
                    self.set_right_motors_velocity(velocity_inner * brake_mult)
                else:
                    self.set_right_motors_velocity(velocity_outer * brake_mult)
                    self.set_left_motors_velocity(velocity_inner * brake_mult)
            else:
                if self.calculate_distance(
                        initial_average_encoder_reading) >= distance - self.linear_precision_pref - self.braking_distance:
                    self.set_right_motors_velocity(self.braking_velocity)
                    self.set_left_motors_velocity(self.braking_velocity)
            self.advance()

    def print_pose(self):
        pose_str = f"Heading:  {self.get_compass_reading()}°"
        pose_str += f" || X: {self.estimated_x:.2f}m"
        pose_str += f" || Y: {self.estimated_y:.2f}m"
        pose_str += f" || F: {self.detect_distance(180):.2f}"
        pose_str += f" || R: {self.detect_distance(-90):.2f}"
        pose_str += f" || L: {self.detect_distance(90):.2f}"
        pose_str += f" || S: {self.state}"
        sys.stdout.write("\r" + pose_str)
        sys.stdout.flush()

    def print_grid(self):
        grid = [[' . ' for i in range(self.grid_dims[0])] for j in range(self.grid_dims[1])]
        for cell in self.visited:
            grid[cell[0]+math.floor(self.grid_dims[0]/2)][cell[1]+math.floor(self.grid_dims[1]/2)] = ' X '
        for cell in self.occupied:
            grid[cell[0]+math.floor(self.grid_dims[0]/2)][cell[1]+math.floor(self.grid_dims[1]/2)] = ' O '
        bot_cell = self.coord_to_cell(self.estimated_x, self.estimated_y)
        grid[bot_cell[0] + math.floor(self.grid_dims[0] / 2)][bot_cell[1] + math.floor(self.grid_dims[1] / 2)] = ' ▪ '
        for j in range(-1, self.grid_dims[1]+1):
            # Print full line
            row = []
            for i in range(-1, self.grid_dims[0]+1):
                if j == -1 or j == self.grid_dims[1]:
                    if i == -1 or i == self.grid_dims[0]:
                        row.append(' + ')
                        continue
                    row.append(' - ')
                    continue
                if i == -1 or i == self.grid_dims[0]:
                    row.append(' | ')
                    continue
                # Add items to row
                row.append(grid[i][j])
            print(''.join(row))
        # Also print estimated coordinates and cell
        print(f"Cell: X: {bot_cell[0]}, Y: {bot_cell[1]}, Confidence: "
              f"{self.cell_probs[bot_cell[0]+math.floor(self.grid_dims[0]/2)][bot_cell[1]+math.floor(self.grid_dims[1]/2)]}\n"
              f"Exact Coordinates: X: {self.estimated_x:.2f}, Y: {self.estimated_y:.2f}, "
              f"θ: {self.get_compass_reading()}")
        return

    def print_probs(self):
        copy = self.cell_probs
        for i in range(self.grid_dims[0]):
            for j in range(self.grid_dims[1]):
                copy[i][j] = round(copy[i][j], 2)
        print(numpy.matrix(numpy.transpose(self.cell_probs)))

    # Advance time (unless stop signal)
    def advance(self):
        if self.step(int(self.getBasicTimeStep())) == -1:
            exit(0)
        self.update_estimates()
        if self.pose_always:
            self.print_pose()
        if self.grid_always:
            self.print_grid()

    # Get relative total distance traveled based on initial encoder readings
    def relative_fle(self):
        return self.get_front_left_motor_encoder_reading() - self.initial_fle

    def relative_fre(self):
        return self.get_front_right_motor_encoder_reading() - self.initial_fre

    # Move forward a given distance (in m), calculated via sensor readings, NOT TIME
    def move_linear(self, distance):
        current_ae = (self.relative_fre() + self.relative_fle()) / 2
        if self.noisy:
            # Print wheel velocities and time estimate
            self.print_decision(distance, self.speed_pref, self.speed_pref)
        # Move forward until average encoder reading is within acceptable
        # range of target distance
        self.set_right_motors_velocity(self.speed_pref)
        self.set_left_motors_velocity(self.speed_pref)
        self.move_until(current_ae, distance)
        self.stop()

    # Move in an arc a given distance (m) with a certain radius (m) - clock direction
    # will be determined by sign of radius:
    # +/- = clockwise/counterclockwise
    def move_arc_distance(self, distance, radius):
        current_ae = (self.relative_fre() + self.relative_fle()) / 2
        sign = 1 if radius >= 0 else 0
        radius = math.fabs(radius)
        velocity_outer = radius + (self.axel_length / 2)
        velocity_inner = radius - (self.axel_length / 2)
        if velocity_inner < 0:
            velocity_outer -= velocity_inner
            velocity_inner -= velocity_inner
        # Normalize velocity to angular speed pref and adjust inner and outer accordingly
        mult = self.angular_speed_pref / velocity_outer
        if sign:
            if self.noisy:
                self.print_decision(distance, velocity_outer * mult, velocity_inner * mult)
            self.set_left_motors_velocity(velocity_outer * mult)
            self.set_right_motors_velocity(velocity_inner * mult)
        else:
            if self.noisy:
                self.print_decision(distance, velocity_inner * mult, velocity_outer * mult)
            self.set_right_motors_velocity(velocity_outer * mult)
            self.set_left_motors_velocity(velocity_inner * mult)
        # Move until distance quota met
        self.move_until(current_ae, distance, True, sign, velocity_outer, velocity_inner)
        self.stop()

    # Traverse an arc, calculating the distance to be traveled based on the angle (deg) and radius (m) passed
    def move_arc_angle(self, angle, radius):
        rads = angle * (math.pi / 180)
        distance = math.fabs(rads * radius)
        self.move_arc_distance(distance, radius)

    # Rotate inplace by a specified angle (in degrees):
    # + clockwise/- counterclockwise
    # Rotation angles above 180deg will be cut or redirected to the minimal path
    def rotate(self, angle):
        self.state = "ROTATE"
        # Rectify input
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        # Ascertain target
        initial_angle = self.get_compass_reading()
        target_angle = initial_angle - angle
        if target_angle > 360:
            target_angle -= 360
        elif target_angle < 0:
            target_angle += 360
        # Rotate depending on sign
        while not (
                target_angle - self.angular_precision_pref < self.get_compass_reading() < target_angle + self.angular_precision_pref):
            if angle > 0:
                self.set_left_motors_velocity(self.rotational_speed_pref)
                self.set_right_motors_velocity(-self.rotational_speed_pref)
            elif angle < 0:
                self.set_left_motors_velocity(-self.rotational_speed_pref)
                self.set_right_motors_velocity(self.rotational_speed_pref)
            self.advance()
        self.stop()

    # Rotate to specific compass reading
    def rotate_to(self, angle):
        self.rotate(self.get_compass_reading() - angle)

    # PID-based functions start here

    # Convert angle from backward in degrees to lidar index
    # Since python can handle negative indices, no need to account for negative angles
    def angle_to_lidar(self, angle):
        return round((angle / 360) * 800)

    # Get (error) value as ratio to upper bound, capped to 1.
    def error_as_ratio(self, error, upper):
        error = error / upper
        if error > 1:
            error = 1
        if error < -1:
            error = -1
        return error

    # Get asymptotic function result for input value - used to extract multiplier from error value
    def asymptotic_error(self, error, pid):
        error *= pid
        return error / (math.sqrt(math.pow(error, 2) + 1))

    # Get lidar readings within specified range
    def get_lidar_ranges(self, angle, width):
        ranges = self.lidar.getRangeImage()
        considered = []
        index_back = self.angle_to_lidar(angle - (width / 2))
        index_forward = self.angle_to_lidar(angle + (width / 2))
        for i in range(index_back, index_forward):
            considered.append(ranges[i])
        return considered

    # Detect average robot-obstacle distance in specified direction
    def detect_distance(self, angle):
        ranges = self.get_lidar_ranges(angle, self.range_width)
        return statistics.mean(ranges)

    # Detect least robot-obstacle distance in specified direction and range
    def detect_closest(self, angle, width=0):
        if not width:
            width = self.range_width
        ranges = self.get_lidar_ranges(angle, width)
        return min(ranges)

    def get_forward_distance(self):
        return self.detect_closest(180, self.range_width)

    # Detect left wall distance
    def get_left_wall_distance(self):
        return self.detect_closest(90, self.range_width)

    # Detect right wall distance
    def get_right_wall_distance(self):
        return self.detect_closest(-90, self.range_width)

    # Get motor speed for moving toward a wall from error and braking distance
    def pid_speed(self, error):
        error *= self.pid_kp
        error = self.error_as_ratio(error, self.braking_distance)
        return error * self.speed_pref

    # Get inner wheel speed for turn based on outer wheel speed and wall error
    def inner_speed(self, outer, error):
        # Outer is max. Error should decrease inner the greater it is
        return outer * (1 - self.asymptotic_error(error, self.pid_ks))

    # Move forward according to specified PID error
    def pid_linear(self, error):
        if error > 0:
            self.set_state("FORWARD")
        elif error < 0:
            self.set_state("REVERSE")
        else:
            self.set_state("STOPPED")
        self.set_left_motors_velocity(self.pid_speed(error))
        self.set_right_motors_velocity(self.pid_speed(error))
        return

    # Turn left to approach wall. Based on last measured wall distance, ensure overturning does not occur.
    def pid_left(self, error):
        # Reverse engineer preferred distance
        distance = self.get_left_wall_distance() - error
        if math.fabs(self.get_forward_distance() - distance) <= self.linear_precision_pref:
            # Too close to forward wall
            self.stop()
            return
        # Positive error means go closer. Negative means get further.
        if error > 0:
            self.set_state("TURN-LEFT")
            # Error is positive. Try to get closer to the wall.
            self.set_right_motors_velocity(self.wall_following_speed)
            self.set_left_motors_velocity(self.wall_following_speed - (math.fabs(error) * self.pid_ks))
        elif error < 0:
            self.set_state("TURN-RIGHT")
            # Error is negative. Try to get further from the wall.
            self.set_left_motors_velocity(self.wall_following_speed)
            self.set_right_motors_velocity(self.wall_following_speed - (math.fabs(error) * self.pid_ks))
            return
        else:
            self.set_state("ANGLE")
            # Error is 0. This will not happen if the function is used properly with precision pref.
            return

    def pid_right(self, error):
        # Reverse engineer preferred distance
        distance = self.get_right_wall_distance() - error
        if math.fabs(self.get_forward_distance() - distance) <= self.linear_precision_pref:
            # Too close to forward wall
            self.stop()
            return
        # Positive error means go closer. Negative means get further.
        if error > 0:
            self.set_state("TURN-RIGHT")
            # Error is positive. Try to get closer to the wall.
            self.set_left_motors_velocity(self.wall_following_speed)
            self.set_right_motors_velocity(self.wall_following_speed - (math.fabs(error) * self.pid_ks))
        elif error < 0:
            self.set_state("TURN-LEFT")
            # Error is negative. Try to get further from the wall.
            self.set_right_motors_velocity(self.wall_following_speed)
            self.set_left_motors_velocity(self.wall_following_speed - (math.fabs(error) * self.pid_ks))
            return
        else:
            self.set_state("ANGLE")
            # Error is 0. This will not happen if the function is used properly with precision pref.
            return

    # Determine if left or right wall is more than 2.5 times input distance away
    def left_far(self, distance):
        return self.get_left_wall_distance() > distance * 2.5

    def left_very_far(self, distance):
        return self.get_left_wall_distance() > distance * 10

    def right_far(self, distance):
        return self.get_right_wall_distance() > distance * 2.5

    def right_very_far(self, distance):
        return self.get_right_wall_distance() > distance * 10

    # Use quarter-circle turns to align with wall
    def align_left(self, distance):
        error = self.get_left_wall_distance() - distance
        radius = error / 2
        self.move_arc_angle(90, -radius)
        self.move_arc_angle(90, radius)

    def align_right(self, distance):
        error = self.get_right_wall_distance() - distance
        radius = error / 2
        self.move_arc_angle(90, radius)
        self.move_arc_angle(90, -radius)

    # Move forward until reaching the input distance from an object
    # Will implement PID control to maximize speed and stop smoothly
    def move_within(self, distance):
        error = self.detect_distance(180) - distance
        if math.fabs(error) <= self.linear_precision_pref:
            self.stop()
            return
        # Cap error to only slow down within braking distance
        self.pid_linear(error)

    # Follow specified wall, attempting to maintain specified distance. Corners will be defined where
    # the forward distance is the preferred wall distance.
    def follow_wall(self, wall="left", distance=0.3):
        if wall == "left":
            # If there is a wall within "distance" in front of me, turn 90 toward larger side opening
            if self.get_forward_distance() <= distance + self.linear_precision_pref or self.left_far(distance):
                if self.get_left_wall_distance() > self.get_right_wall_distance() or self.left_far(distance):
                    # If left and right walls appear very far, rotate 90deg right and align with forward wall
                    if self.left_very_far(distance) and self.right_very_far(
                            distance) and self.get_forward_distance() < distance * 5:
                        err_for = self.get_forward_distance() - distance
                        if math.fabs(err_for) <= self.linear_precision_pref * 10:
                            self.rotate(90)
                        else:
                            self.move_linear(err_for)
                        return
                    # Turn toward left opening
                    if self.left_far(distance):
                        self.move_linear(distance / 2)
                    self.rotate(-90)
                    if self.left_far(distance):
                        self.move_linear(distance * 2)
                    else:
                        self.move_linear(distance)
                    if not self.left_far(distance):
                        self.align_left(distance)
                else:
                    # Turn toward right opening
                    self.rotate(90)
                    self.move_linear(distance)
                    self.align_left(distance)
            # A positive error means we must get closer
            error = self.get_left_wall_distance() - distance
            # If we are very close to the wall, rotate away slightly as a failsafe.
            if self.get_left_wall_distance() < 0.2:
                self.rotate(10)
                self.move_linear(0.05)
            self.pid_left(error)
        else:
            if self.get_forward_distance() <= distance + self.linear_precision_pref or self.right_far(distance):
                # If left and right walls appear very far, rotate 90deg left and align with forward wall
                if self.left_very_far(distance) and self.right_very_far(distance) and self.get_forward_distance() < distance * 5:
                    err_for = self.get_forward_distance() - distance
                    if math.fabs(err_for) <= self.linear_precision_pref * 10:
                        self.rotate(-90)
                    else:
                        self.move_linear(err_for)
                    return
                if self.right_far(distance):
                    # Turn toward right opening
                    self.move_linear(distance / 2)
                    self.rotate(90)
                    self.move_linear(distance * 2)
                    if not self.right_far(distance):
                        self.align_right(distance)
                elif self.get_left_wall_distance() > self.get_right_wall_distance():
                    # Turn toward left opening
                    self.rotate(-90)
                    self.move_linear(distance)
                    self.align_right(distance)
                else:
                    # Turn toward right opening
                    self.rotate(90)
                    self.move_linear(distance)
                    self.align_right(distance)
            error = self.get_right_wall_distance() - distance
            # If we are very close to the wall, rotate away slightly as a failsafe.
            if self.get_right_wall_distance() < 0.2:
                self.rotate(-10)
                self.move_linear(0.05)
            self.pid_right(error)

    # Move forward until reaching input distance from an object
    # Also avoid side walls by input distance
    def move_through(self, distance, wall_distance):
        error_forward = self.detect_distance(180) - distance
        error_left = self.get_left_wall_distance() - wall_distance
        error_right = self.get_right_wall_distance() - wall_distance
        # Stop if within range of final destination
        if math.fabs(error_forward) <= self.linear_precision_pref:
            self.stop()
            return
        if self.get_left_wall_distance() >= wall_distance and self.get_right_wall_distance() >= wall_distance:
            # If we are far enough from both walls, simply move forward
            self.move_within(distance)
        elif math.fabs(error_left) > math.fabs(error_right):
            # Move away/toward the left wall
            if error_left > 0:
                # Move left
                self.pid_left(error_left)
            else:
                # Move right
                self.pid_right(error_left)
        else:
            # Move away/toward the right wall
            if error_right > 0:
                # Move right
                self.pid_right(error_right)
            else:
                # Move left
                self.pid_left(error_right)

    # Start of Camera-Based Functions

    # Attempt to find target object.
    # Returns an array containing relative distance and angle to object
    def find_target(self):
        rec_all = self.rgb_camera.getRecognitionObjects()
        if len(rec_all) > 0:
            # An object is found
            target = rec_all[0]
            # Attempt to ascertain relative angle.
            # Important: detection begins at position 3, not 0.
            # Total camera FOV is 62deg
            position_horizontal = target.getPositionOnImage()[0] - 3
            ratio_horizontal = position_horizontal / self.camera_resolution[0]
            angle_from_right = self.camera_fov[0] * ratio_horizontal
            angle_from_forward = angle_from_right - self.camera_fov[0] / 2
            # Now attempt to ascertain distance to target.
            # Knowing the size of sought target, the relative size can be used
            # As this will be done with horizontal measurements, partial occlusion from walls is no problem.
            size_horizontal = target.getSizeOnImage()[0]
            # Get visual angle of target
            # This will be sensitive to the fish-eye effect, but this will be ignored.
            # This will be naturally handled by rotation toward the object.
            ratio_horizontal_size = size_horizontal / self.camera_resolution[0]
            target_visual_angle = self.camera_fov[0] * ratio_horizontal_size
            # Using half of the visual angle, construct right triangle with true size as opposite and d as adjacent
            half_angle = target_visual_angle / 2
            half_size = self.target_size[1] / 2
            distance = math.fabs(half_size / math.tan(half_angle * (math.pi / 180)))
            # Weird
            target_colors = target.getColors()
            r = round(target_colors[0])
            g = round(target_colors[1])
            b = round(target_colors[2])
            color = [r, g, b]
            # Correction factor of -0.11127738671450094
            return [distance - 0.11127738671450094, angle_from_forward, color]
        return []

    def approach_target(self, distance=0.3):
        target = self.find_target()
        if not len(target) > 0:
            # Target not found at all. Rotate until visible.
            self.rotate(self.camera_fov[0] / 2)
        else:
            # Target found. Adjust angle to within reason and approach
            target_angle = target[1]
            error = target[0] - distance
            if math.fabs(target_angle) > (self.target_angle_width / 2):
                self.rotate(target_angle)
            if error > self.linear_precision_pref:
                self.pid_linear(math.fabs(error))
            else:
                self.stop()

    def bug_zero(self, wall="left", distance=0.3):
        target = self.find_target()
        if not len(target) > 0:
            # Target not found at all. Wall follow until visible
            self.follow_wall(wall, distance)
        elif target[0] > self.detect_distance(180 + target[1]):
            self.follow_wall(wall, distance)
        else:
            # Target found. Adjust angle to within reason and approach
            self.approach_target(distance)

    # Start of Localization Functions

    def p_to_logsodd(self, p):
        if p >= 1:
            return math.inf
        return numpy.log(p / (1 - p))

    def logsodd_to_p(self, logsodd):
        return 1 - (1 / (1 + math.pow(math.e, logsodd)))

    # Convert the input x and y coordinates to a cell coordinate on the map grid, bearing in mind:
    # 1 cell is defined by self.cell_len
    # The center of the grid is defined at 0,0 (intersection between cells at [1,1],[-1,1],[-1,-1],[-1,-1]
    # Up = Negative
    # Right = Positive
    def coord_to_cell(self, x, y):
        cell_x = math.floor(x / self.cell_len)
        cell_y = math.floor(-y / self.cell_len)
        return tuple([cell_x, cell_y])

    # Check if cell is in bounds
    def cell_in_bounds(self, cell):
        left = -math.floor(self.grid_dims[0] / 2)
        right = math.floor(self.grid_dims[0] / 2) - 1
        top = -math.floor(self.grid_dims[1] / 2)
        bottom = math.floor(self.grid_dims[1] / 2) - 1
        x = cell[0]
        y = cell[1]
        return not (x < left or x > right or y < top or y > bottom)

    # Check if cell is visited
    def cell_visited(self, cell):
        return tuple(cell) in frozenset(self.visited)

    # Check if cell is occupied
    def cell_occupied(self, cell):
        return tuple(cell) in frozenset(self.occupied)

    # Check if cell is unoccupied and unvisited and in bounds
    def cell_open(self, cell):
        return (not (self.cell_visited(cell) or self.cell_occupied(cell))) and self.cell_in_bounds(cell)

    # Snap to nearest cardinal direction
    def snap_cardinal(self):
        self.rotate_to(self.degrees_from_cardinal(self.get_cardinal()))

    # Get the current closest cardinal position
    def get_cardinal(self):
        return self.cardinal_from_degrees(self.get_compass_reading())

    def cardinal_from_degrees(self, degrees):
        rounded = round(degrees / 90)
        if rounded == 0 or rounded == 4:
            return 'E'
        elif rounded == 1:
            return 'N'
        elif rounded == 2:
            return 'W'
        elif rounded == 3:
            return 'S'

    # Get degrees from cardinal direction
    def degrees_from_cardinal(self, cardinal):
        if cardinal == 'N':
            return 90
        elif cardinal == 'E':
            return 0
        elif cardinal == 'S':
            return 270
        elif cardinal == 'W':
            return 180

    # Check if there's a wall in the specified cardinal direction
    def wall_in_direction(self, dir='N'):
        self.snap_cardinal()
        current = self.get_compass_reading()
        dir_angle = self.degrees_from_cardinal(dir)
        dif = dir_angle - current
        while dif > 360:
            dif -= 360
        while dif < 0:
            dif += 360
        # now dif represents the angle from the robot that we are trying to check
        # relative_cardinal is the direction from the robot, where N is right
        relative_cardinal = self.cardinal_from_degrees(dif)
        if relative_cardinal == 'S':
            return self.wall_right()
        elif relative_cardinal == 'W':
            return self.wall_behind()
        elif relative_cardinal == 'N':
            return self.wall_left()
        elif relative_cardinal == 'E':
            return self.wall_front()

    # Get angles and colors of all visible landmarks
    def find_landmarks(self):
        # Orient north, then rotate fully, remembering landmark distances
        landmarks = []
        self.rotate_to(90)
        for i in range(360):
            # Seek landmarks. If we see one, update its compass angle from our position
            # Do this only when the object is centered to avoid fisheye artifacts as much as possible
            target = self.find_target()
            if target:
                color = target[2]
                angle = target[1]
                distance = target[0]
                # Determine which landmark this is and its coordinates
                coords = []
                for landmark in self.landmarks:
                    if landmark[0] == color:
                        coords = landmark[1]
                        break
                if math.fabs(angle) < self.angular_precision_pref:
                    new_landmark = [coords, distance, angle]
                    already = 0
                    for landmark in landmarks:
                        if landmark[0] == new_landmark[0]:
                            already = 1
                    if not already:
                        landmarks.append(new_landmark)
            self.rotate_to(90+i)
        # Re-orient north since we may be slightly off
        self.rotate_to(90)
        return landmarks

    # Using predefined map information (grid dimensions, landmark positions,
    # determine which cell we are in, and where in that cell
    def trilaterate(self):
        # If there are less than 3 landmarks, give up, since we cannot use angles.
        landmarks = self.find_landmarks()
        if len(landmarks) < 3:
            return
        # We have 3 landmarks (at least). Separate them into usable bits
        L1 = landmarks[0]
        L2 = landmarks[1]
        L3 = landmarks[2]
        # Circle information for each [cx, cy, r]
        C1 = [L1[0][0], L1[0][1], L1[1]]
        C2 = [L2[0][0], L2[0][1], L2[1]]
        C3 = [L3[0][0], L3[0][1], L3[1]]
        # ABCDEF according to Trilateration algorithm described in slide
        A = (-2 * C1[0]) + (2 * C2[0])
        B = (-2 * C1[1]) + (2 * C2[1])
        C = math.pow(C1[2], 2) - math.pow(C2[2], 2) - math.pow(C1[0], 2) + math.pow(C2[0], 2) - math.pow(C1[1], 2) + math.pow(C2[1], 2)
        D = (-2 * C2[0]) + (2 * C3[0])
        E = (-2 * C2[1]) + (2 * C3[1])
        F = math.pow(C2[2], 2) - math.pow(C3[2], 2) - math.pow(C2[0], 2) + math.pow(C3[0], 2) - math.pow(C2[1], 2) + math.pow(C3[1], 2)
        # Exception as stated in slide:
        if E * A == B * D:
            return []
        # Final calculation
        x = ((C*E) - (F*B)) / ((E*A) - (B*D))
        y = ((C*D) - (A*F)) / ((B*D) - (A*E))
        self.estimated_x = x
        self.estimated_y = y
        return tuple([x, y])

    # Navigate one cell in any particular direction
    def move_cell_left(self):
        self.rotate_to(180)
        self.move_linear(self.cell_len)
        self.previous_moves.append('L')

    def move_cell_right(self):
        self.rotate_to(0)
        self.move_linear(self.cell_len)
        self.previous_moves.append('R')

    def move_cell_up(self):
        self.rotate_to(90)
        self.move_linear(self.cell_len)
        self.previous_moves.append('U')

    def move_cell_down(self):
        self.rotate_to(270)
        self.move_linear(self.cell_len)
        self.previous_moves.append('D')

    # Undo last known cell movement and return what it was
    def undo_last_cell_move(self):
        last = self.previous_moves[len(self.previous_moves)-1]
        self.previous_moves.pop()
        if last == 'L':
            self.move_cell_right()
        elif last == 'R':
            self.move_cell_left()
        elif last == 'U':
            self.move_cell_down()
        elif last == 'D':
            self.move_cell_up()
        self.previous_moves.pop()
        return last

    # Calculate a path through every grid cell while avoiding pre-visited or occupied cells
    def navigate_grid(self):
        start = self.coord_to_cell(self.estimated_x, self.estimated_y)
        left = [start[0]-1, start[1]]
        up = [start[0], start[1]-1]
        down = [start[0], start[1]+1]
        right = [start[0]+1, start[1]]
        # It goes like this:
        # Routine: Avoid all obstacles AND visited spaces
        #   1. Left?
        #   2. Up?
        #   3. Down?
        #   4. Right?
        #   5. If none, undo last move and try again
        # Repeat until the sum of visited and occupied cells is all cells
        while len(self.visited) + len(self.occupied) < self.grid_dims[0] * self.grid_dims[1]:
            cell = self.coord_to_cell(self.estimated_x, self.estimated_y)
            self.visited.add(cell)
            self.print_grid()
            if self.cell_open(left):
                self.move_cell_left()
                left[0] -= 1
                up[0] -= 1
                down[0] -= 1
                right[0] -= 1
            elif self.cell_open(up):
                self.move_cell_up()
                left[1] -= 1
                up[1] -= 1
                down[1] -= 1
                right[1] -= 1
            elif self.cell_open(down):
                self.move_cell_down()
                left[1] += 1
                up[1] += 1
                down[1] += 1
                right[1] += 1
            elif self.cell_open(right):
                self.move_cell_right()
                left[0] += 1
                up[0] += 1
                down[0] += 1
                right[0] += 1
            else:
                last = self.undo_last_cell_move()
                if last == 'L':
                    left[0] += 1
                    up[0] += 1
                    down[0] += 1
                    right[0] += 1
                elif last == 'R':
                    left[0] -= 1
                    up[0] -= 1
                    down[0] -= 1
                    right[0] -= 1
                elif last == 'U':
                    left[1] += 1
                    up[1] += 1
                    down[1] += 1
                    right[1] += 1
                elif last == 'D':
                    left[1] -= 1
                    up[1] -= 1
                    down[1] -= 1
                    right[1] -= 1
        self.print_grid()

    # Detect whether a wall is present in a given direction
    def wall_behind(self):
        return self.detect_closest(0, 10) < self.cell_len

    def wall_left(self):
        return self.detect_closest(90, 10) < self.cell_len

    def wall_front(self):
        return self.detect_closest(180, 10) < self.cell_len

    def wall_right(self):
        return self.detect_closest(-90, 10) < self.cell_len

    # Determine the wall characteristics of the current cell, with confidence level
    def detect_walls(self):
        walls = ""
        confidence = self.p_to_logsodd(0.5)
        if self.wall_in_direction('S'):
            walls += 'S'
            confidence += 0.9
        else:
            confidence += 0.7
        if self.wall_in_direction('W'):
            walls += 'W'
            confidence += 0.9
        else:
            confidence += 0.7
        if self.wall_in_direction('N'):
            walls += 'N'
            confidence += 0.9
        else:
            confidence += 0.7
        if self.wall_in_direction('E'):
            walls += 'E'
            confidence += 0.9
        else:
            confidence += 0.7
        return [walls, self.logsodd_to_p(confidence)]

    # Update cell probability
    def update_cell_prob(self, x, y, new_prob, force=False):
        if force:
            self.cell_probs[x][y] = new_prob
            return
        self.cell_probs[x][y] = self.logsodd_to_p(self.p_to_logsodd(self.cell_probs[x][y]) -
                                                                    self.p_to_logsodd(0.5) +
                                                                    self.p_to_logsodd(new_prob))

    # Assess which cell we are probably in
    def wall_estimate_cell(self):
        walls = self.detect_walls()
        dirs = walls[0]
        confidence = walls[1]
        # Obtain potential matches
        matches = []
        for i in range(self.grid_dims[0]):
            for j in range(self.grid_dims[1]):
                if self.cell_walls[i][j] == dirs:
                    matches.append([i, j])
        # Update the probabilities for each match
        for cell in matches:
            self.update_cell_prob(cell[0], cell[1], confidence)
        self.normalize_probs()

    # Get most likely cell. If many are tied, take any.
    def guess_cell(self):
        index = numpy.argmax(self.cell_probs)
        x = int(math.floor(index / self.grid_dims[0]) - (self.grid_dims[0] / 2))
        y = int(index % self.grid_dims[1] - (self.grid_dims[1] / 2))
        # Update coordinates according to cell guess
        self.estimated_x = x * self.cell_len
        self.estimated_y = -y * self.cell_len
        return tuple([x, y])

    # Normalize probs to sum to 1
    def normalize_probs(self):
        sum = numpy.sum(self.cell_probs)
        for i in range(self.grid_dims[0]):
            for j in range(self.grid_dims[1]):
                self.cell_probs[i][j] /= sum

    # Assert starting position with 100% confidence
    def assert_probs_position(self, x, y):
        for i in range(self.grid_dims[0]):
            for j in range(self.grid_dims[1]):
                self.cell_probs[i][j] = 0
        self.cell_probs[x][y] = 1

    # Move one cell length in specified compass direction while applying motion model to cell probs
    def move_probably(self, dir='N'):
        current = self.guess_cell()
        x = current[0] - math.floor(self.grid_dims[0] / 2)
        y = current[1] - math.floor(self.grid_dims[1] / 2)
        if dir == 'N':
            self.move_cell_up()
            self.update_cell_prob(x, y, 0.2, True)
            self.update_cell_prob(x, y - 1, 0.8, True)
        elif dir == 'E':
            self.move_cell_right()
            self.update_cell_prob(x, y, 0.2, True)
            self.update_cell_prob(x + 1, y, 0.8, True)
        elif dir == 'S':
            self.move_cell_down()
            self.update_cell_prob(x, y, 0.2, True)
            self.update_cell_prob(x, y + 1, 0.8, True)
        elif dir == 'W':
            self.move_cell_left()
            self.update_cell_prob(x, y, 0.2, True)
            self.update_cell_prob(x - 1, y, 0.8, True)
        self.normalize_probs()
        self.guess_cell()

    # Navigate based on position probabilities and known visited cells
    # This algorithm will be pretty much identical to the open grid one, but also checking for walls
    # and guessing the cell we're in.
    # This guessing may lead to "teleportation" where the bot suddenly changes its estimated position,
    # but the visitation memory and undoing routine should allow for visiting cells of broken branches anyway.
    # Importantly as well, these "teleportations" must also translate the list of visited cells congruently.
    def navigate_wall_probs(self):
        self.wall_estimate_cell()
        start = self.guess_cell()
        left = [start[0] - 1, start[1]]
        up = [start[0], start[1] - 1]
        down = [start[0], start[1] + 1]
        right = [start[0] + 1, start[1]]
        # It goes like this:
        # Routine: Avoid all obstacles AND visited spaces
        #   1. Left?
        #   2. Up?
        #   3. Down?
        #   4. Right?
        #   5. If none, undo last move and try again
        # Repeat until the sum of visited and occupied cells is all cells
        while len(self.visited) + len(self.occupied) < self.grid_dims[0] * self.grid_dims[1]:
            self.wall_estimate_cell()  # doing this just reduces confidence, but technically helps if the robot is moved
            self.visited.add(self.guess_cell())
            self.print_grid()
            self.print_probs()
            if self.cell_open(left) and not self.wall_in_direction('W'):
                self.move_probably('W')
                left[0] -= 1
                up[0] -= 1
                down[0] -= 1
                right[0] -= 1
            elif self.cell_open(up) and not self.wall_in_direction('N'):
                self.move_probably('N')
                left[1] -= 1
                up[1] -= 1
                down[1] -= 1
                right[1] -= 1
            elif self.cell_open(down) and not self.wall_in_direction('S'):
                self.move_probably('S')
                left[1] += 1
                up[1] += 1
                down[1] += 1
                right[1] += 1
            elif self.cell_open(right) and not self.wall_in_direction('E'):
                self.move_probably('E')
                left[0] += 1
                up[0] += 1
                down[0] += 1
                right[0] += 1
            else:
                last = self.undo_last_cell_move()
                if last == 'L':
                    left[0] += 1
                    up[0] += 1
                    down[0] += 1
                    right[0] += 1
                elif last == 'R':
                    left[0] -= 1
                    up[0] -= 1
                    down[0] -= 1
                    right[0] -= 1
                elif last == 'U':
                    left[1] += 1
                    up[1] += 1
                    down[1] += 1
                    right[1] += 1
                elif last == 'D':
                    left[1] -= 1
                    up[1] -= 1
                    down[1] -= 1
                    right[1] -= 1
