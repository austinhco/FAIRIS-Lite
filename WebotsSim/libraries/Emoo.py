# WebotsSim/libraries/Emoo.py

import math
import statistics
import sys

from WebotsSim.libraries.RobotLib.RosBot import RosBot


class MyRobot(RosBot):

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
    wall_error_precision_pref = 0.1 # meters
    braking_distance = 0.25  # meters
    braking_velocity = 1  # radians per second per wheel
    angular_braking_velocity = 1  # radians per second per wheel
    wall_avoidance_aggression = 1.2  # distance (m) where turning slows (less aggressive with larger values)

    # Sensor Preference Parameters
    range_width = 30  # Width of lidar readings in degrees (averages values)

    # PID Preference Parameters
    pid_k = 1

    # Basal Sensor Readings
    initial_fle = 0
    initial_fre = 0

    # Position estimates
    estimated_x = 0
    estimated_y = 0

    # Last encoder readings
    last_fle = 0
    last_fre = 0

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
        return (statistics.mean([self.relative_fre(), self.relative_fle()]) - initial_average_encoder_reading) * self.wheel_radius

    def print_decision(self, distance, velocity_l, velocity_r):
        speed = statistics.mean([velocity_l, velocity_r])
        print("     Speed: " + str(round(speed*self.wheel_radius, 2)) + "m/s")
        print("L Velocity: " + str(round(velocity_l*self.wheel_radius, 2)) + "m/s")
        print("R Velocity: " + str(round(velocity_r*self.wheel_radius, 2)) + "m/s")
        print("       ETA: " + str(round((distance / (speed * self.wheel_radius)) + (
                    self.braking_distance / (self.braking_velocity * self.wheel_radius)), 2)) + "s")

    # Move until a set distance has been travelled. If arc is set, brake as such using movement sign
    def move_until(self, initial_average_encoder_reading, distance, arc=False, sign=0, velocity_outer=0, velocity_inner=0):
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
                if self.calculate_distance(initial_average_encoder_reading) >= distance - self.linear_precision_pref - self.braking_distance:
                    self.set_right_motors_velocity(self.braking_velocity)
                    self.set_left_motors_velocity(self.braking_velocity)
            self.advance()

    def print_pose(self):
        pose_str =  f"Heading:  {self.get_compass_reading()}°"
        pose_str += f" || X: {self.estimated_x:.2f}m"
        pose_str += f" || Y: {self.estimated_y:.2f}m"
        pose_str += f" || F: {self.detect_distance(180):.2f}"
        pose_str += f" || R: {self.detect_distance(90):.2f}"
        pose_str += f" || L: {self.detect_distance(-90):.2f}"
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
        # Hack away a silly little precision error
        # distance += (math.pi / 180)
        current_ae = (self.relative_fre() + self.relative_fle()) / 2
        sign = 1 if radius >= 0 else 0
        radius = math.fabs(radius)
        velocity_outer = radius + (self.axel_length/2)
        velocity_inner = radius - (self.axel_length/2)
        # Normalize velocity to angular speed pref and adjust inner and outer accordingly
        mult = self.angular_speed_pref / statistics.mean([velocity_outer, velocity_inner])
        if sign:
            if self.noisy:
                self.print_decision(distance, velocity_outer*mult, velocity_inner*mult)
            self.set_left_motors_velocity(velocity_outer * mult)
            self.set_right_motors_velocity(velocity_inner * mult)
        else:
            if self.noisy:
                self.print_decision(distance, velocity_inner*mult, velocity_outer*mult)
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
        while not (target_angle - self.angular_precision_pref < self.get_compass_reading() < target_angle + self.angular_precision_pref):
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
        return round((angle/360) * 800)

    # Cap input PID error value to within braking distance preference
    def cap_error(self, error, max):
        error = error / max
        if error > 1:
            error = 1
        if error < -1:
            error = -1
        return error

    # Detect distance between robot and nearest object in specified direction (deg)
    def detect_distance(self, angle):
        ranges = self.lidar.getRangeImage()
        considered = []
        index_back = self.angle_to_lidar(angle-(self.range_width/2))
        index_forward = self.angle_to_lidar(angle+(self.range_width/2))
        for i in range(index_back, index_forward):
            considered.append(ranges[i])
        return statistics.mean(considered)

    # Move forward until reaching the input distance from an object
    # Will implement PID control to maximize speed and stop smoothly
    def move_within(self, distance):
        error = (self.detect_distance(180) - distance) * self.pid_k
        if math.fabs(error) <= self.linear_precision_pref:
            self.stop()
            return
        # Cap error to only slow down within braking distance
        error = self.cap_error(error, self.braking_distance)
        self.set_left_motors_velocity(error*self.max_motor_velocity)
        self.set_right_motors_velocity(error*self.max_motor_velocity)

    # Move forward until reaching input distance from an object
    # Also avoid side walls by input distance
    def move_through(self, distance, wall_distance):
        error = (self.detect_distance(180) - distance) * self.pid_k
        error_left = (self.detect_distance(90) - wall_distance) * self.pid_k
        error_right = (self.detect_distance(-90) - wall_distance) * self.pid_k
        # Stop if within range of final destination
        if math.fabs(error) <= self.linear_precision_pref:
            self.stop()
            return
        # Cap error within braking distance
        error = self.cap_error(error, self.braking_distance)
        if error_left >= wall_distance and error_right >= wall_distance:
            # If we are far enough from both walls, simply move forward
            self.set_left_motors_velocity(error * self.max_motor_velocity)
            self.set_right_motors_velocity(error * self.max_motor_velocity)
            return
        elif math.fabs(error_left) > math.fabs(error_right):
            # Move away/toward the left wall
            if error_left > 0:
                # Move left
                self.set_right_motors_velocity(error*self.max_motor_velocity)
                error_left = self.cap_error(error_left, self.wall_avoidance_aggression)
                self.set_left_motors_velocity(error*self.max_motor_velocity * (1 - error_left))
                return
            else:
                # Move right
                self.set_left_motors_velocity(error*self.max_motor_velocity)
                error_left = self.cap_error(error_left, self.wall_avoidance_aggression)
                self.set_right_motors_velocity(error*self.max_motor_velocity * (1 - error_left))
                return
        else:
            # Move away/toward the right wall
            if error_right > 0:
                # Move right
                self.set_left_motors_velocity(error*self.max_motor_velocity)
                error_right = self.cap_error(error_right, self.wall_avoidance_aggression)
                self.set_right_motors_velocity(error*self.max_motor_velocity * (1 - error_right))
                return
            else:
                # Move left
                self.set_right_motors_velocity(error*self.max_motor_velocity)
                error_right = self.cap_error(error_right, self.wall_avoidance_aggression)
                self.set_left_motors_velocity(error*self.max_motor_velocity * (1 - error_right))
                return
