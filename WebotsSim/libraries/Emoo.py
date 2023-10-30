# WebotsSim/libraries/Emoo.py

import math
import statistics
import sys

from WebotsSim.libraries.RobotLib.RosBot import RosBot


class Emoo(RosBot):

    def __init__(self):
        RosBot.__init__(self)

    # Print velocities and times before moving
    noisy = False

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

    # Last encoder readings
    last_fle = 0
    last_fre = 0

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

    # Advance time (unless stop signal)
    def advance(self, quiet=False):
        if self.step(int(self.getBasicTimeStep())) == -1:
            exit(0)
        self.update_estimates()
        if not quiet:
            self.print_pose()

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
            self.advance(True)
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
        return error/(math.sqrt(math.pow(error, 2)+1))

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
        return error * self.max_motor_velocity

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

    def right_far(self, distance):
        return self.get_right_wall_distance() > distance * 2.5

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
                    # Turn toward left opening
                    if self.left_far(distance):
                        self.move_linear(distance/2)
                    self.rotate(-90)
                    if self.left_far(distance):
                        self.move_linear(distance*2)
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
            self.pid_left(error)
        else:
            if self.get_forward_distance() <= distance + self.linear_precision_pref or self.right_far(distance):
                if self.right_far(distance):
                    # Turn toward right opening
                    self.move_linear(distance/2)
                    self.rotate(90)
                    self.move_linear(distance*2)
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

    # # Turn left according to specified PID error (forward and left)
    # def pid_left(self, error_forward, error_left):
    #     error_forward *= self.pid_k
    #     error_left *= self.pid_k
    #     error_forward = self.cap_error(error_forward, self.braking_distance)
    #     error_left = self.cap_error(error_left, self.wall_following_aggression)
    #     self.set_right_motors_velocity(error_forward * self.max_motor_velocity)
    #     error_left = self.cap_error(error_left, self.wall_following_aggression)
    #     self.set_left_motors_velocity(error_forward * self.max_motor_velocity * (1 - error_left))
    #     return
    #
    # # Turn right according to specified PID error (forward and right)
    # def pid_right(self, error_forward, error_right):
    #     error_forward *= self.pid_k
    #     error_right *= self.pid_k
    #     error_forward = self.cap_error(error_forward, self.braking_distance)
    #     error_right = self.cap_error(error_right, self.wall_following_aggression)
    #     self.set_left_motors_velocity(error_forward * self.max_motor_velocity)
    #     error_right = self.cap_error(error_right, self.wall_following_aggression)
    #     self.set_right_motors_velocity(error_forward * self.max_motor_velocity * (1 - error_right))
    #     return
    #
    # def get_left_wall_distance(self):
    #     return self.detect_closest(90, 60)
    #
    # def get_right_wall_distance(self):
    #     return self.detect_closest(-90, 60)
    #
    # def correct_left_wall_distance(self, error):
    #     error *= self.pid_k
    #     radius = error / 2
    #     self.move_arc_angle(90, -radius)
    #     self.move_arc_angle(90, radius)
    #     return
    #
    # def correct_right_wall_distance(self, error):
    #     return
    #
    # def correct_wall_distance(self, wall, pref):
    #     if wall == "left":
    #         error = self.get_left_wall_distance() - pref
    #         print("\nerror")
    #         print(error)
    #         if math.fabs(error) > self.wall_error_precision_pref:
    #             print("correcting")
    #             self.correct_left_wall_distance(error)
    #         else:
    #             self.move_within(pref)
    #         return
    #     elif wall == "right":
    #         error = self.get_right_wall_distance() - pref
    #         self.correct_right_wall_distance(error)
    #         return
    #
    #
    #
    # # Follow specified wall at specified distance.
    # # Will make 90 degree turns against the specified wall when encountering a corner
    # def follow_wall(self, wall="left", distance=0.3):
    #     self.correct_wall_distance(wall, distance)
    #
    #     # error_forward = self.detect_distance(180) - distance
    #     # error_left = self.detect_distance(90) - distance
    #     # error_right = self.detect_distance(-90) - distance
    #     # if wall == "left":
    #     #     # Follow left wall... duh
    #     #     if error_forward <= 0 + self.linear_precision_pref:
    #     #         self.rotate(90)
    #     #     else:
    #     #         # If within wall_error_precision_pref, continue forward.
    #     #         # Otherwise, turn slightly in the appropriate direction
    #     #         if error_left > self.wall_error_precision_pref:
    #     #             # Turn toward wall
    #     #             self.pid_left(error_forward, error_left)
    #     #         elif error_left < -self.wall_error_precision_pref:
    #     #             # Turn away from wall
    #     #             self.pid_right(error_forward, error_left)
    #     #         else:
    #     #             # Forward
    #     #             self.pid_forward(1)
    #     # elif wall == "right":
    #     #     # Follow right wall... duh
    #     #     if error_forward <= 0 + self.linear_precision_pref:
    #     #         print(error_forward)
    #     #         self.rotate(-90)
    #     #     else:
    #     #         # Same stuff, other direction
    #     #         if error_right > self.wall_error_precision_pref:
    #     #             self.pid_right(error_forward, error_right)
    #     #         elif error_right < -self.wall_error_precision_pref:
    #     #             print("LEFT")
    #     #             print(error_right)
    #     #             self.pid_left(error_forward, error_right)
    #     #         else:
    #     #             self.pid_forward(1)
    #     # else:
    #     #     print("What")
