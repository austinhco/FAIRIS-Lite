# WebotsSim/libraries/Emoo.py

import math
import statistics
import sys

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
    visited = set([])  # list of visited cells

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
        # Update visited cells if we are in a new one
        cell = self.coord_to_cell(self.estimated_x, self.estimated_y)
        self.visited.add(cell)
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
        return

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
        self.state = "ROTATE"
        while angle > 360:
            angle -= 360
        while angle <= 0:
            angle += 360
        # set rotation direction
        dir = self.get_compass_reading() + 180 < angle
        while (not self.get_compass_reading() == angle) or \
                (angle == 360 and not (self.get_compass_reading() == 0 or self.get_compass_reading() == 360)):
            if dir:
                self.set_left_motors_velocity(1)
                self.set_right_motors_velocity(-1)
            else:
                self.set_left_motors_velocity(-1)
                self.set_right_motors_velocity(1)
            self.advance()
        self.stop()

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

    # Convert the input x and y coordinates to a cell coordinate on the map grid, bearing in mind:
    # 1 cell is defined by self.cell_len
    # The center of the grid is defined at 0,0 (intersection between cells at [1,1],[-1,1],[-1,-1],[-1,-1]
    # Up = Negative
    # Right = Positive
    def coord_to_cell(self, x, y):
        cell_x = math.floor(x / self.cell_len)
        cell_y = math.floor(-y / self.cell_len)
        return tuple([cell_x, cell_y])

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
    def triangulate(self):
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
