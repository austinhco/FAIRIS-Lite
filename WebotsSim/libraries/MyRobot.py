from WebotsSim.libraries.RobotLib.RosBot import RosBot


class MyRobot(RosBot):

    def __init__(self):
        RosBot.__init__(self)

    # Preference parameters
    speed_pref = 5  # radians per second per wheel
    angular_speed_pref = 2  # radians per second relative to ICC
    rotational_speed_pref = 1  # radians per second per wheel
    linear_precision_pref = 0.005  # meters
    angular_precision_pref = 1  # degrees

    # Basal Sensor Readings
    initial_fle = 0
    initial_fre = 0

    # Advance time (unless stop signal)
    def advance(self):
        if self.step(int(self.getBasicTimeStep())) == -1:
            exit(0)

    # Get relative total distance traveled based on initial encoder readings
    def relative_fle(self):
        return self.get_front_left_motor_encoder_reading() - self.initial_fle

    def relative_fre(self):
        return self.get_front_right_motor_encoder_reading() - self.initial_fre

    # Move forward a given distance (in m), calculated via sensor readings, NOT TIME
    def move_linear(self, distance):
        current_ae = (self.relative_fre() + self.relative_fle()) / 2
        # Move forward until average encoder reading is within acceptable
        # range of target distance
        self.set_right_motors_velocity(self.speed_pref)
        self.set_left_motors_velocity(self.speed_pref)
        while ((self.relative_fre() + self.relative_fle())/2 - current_ae) * self.wheel_radius <= distance - self.linear_precision_pref:
            self.advance()
        self.stop()

    # Move in an arc a given distance with a certain radius - clock direction
    # will be determined by sign of radius:
    # +/- = clockwise/counterclockwise
    def move_arc(self, distance, radius):
        current_ae = (self.relative_fre() + self.relative_fle()) / 2
        sign = 1 if radius >= 0 else -1
        if radius < 0:
            radius *= -1
        velocity_outer = self.angular_speed_pref * (radius + (self.axel_length/2))
        velocity_inner = self.angular_speed_pref * (radius - (self.axel_length/2))

        return

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
            self.advance()
        self.stop()



