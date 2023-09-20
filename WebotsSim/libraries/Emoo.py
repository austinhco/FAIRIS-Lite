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

    # Preference parameters
    speed_pref = 5  # radians per second per wheel
    angular_speed_pref = 2  # radians per second relative to ICC
    rotational_speed_pref = 1  # radians per second per wheel
    linear_precision_pref = 0.001  # meters
    angular_precision_pref = 1  # degrees
    braking_distance = 0.25  # meters
    braking_velocity = 1  # radians per second per wheel
    angular_braking_velocity = 1  # radians per second per wheel

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
        pose_str =  "Heading: " + str(self.get_compass_reading()) + "°"
        pose_str += " || X: " + str(round(self.estimated_x, 2)) + "m"
        pose_str += " || Y: " + str(round(self.estimated_y, 2)) + "m"
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



