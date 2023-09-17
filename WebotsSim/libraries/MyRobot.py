from WebotsSim.libraries.RobotLib.RosBot import RosBot


class MyRobot(RosBot):

    def __init__(self):
        RosBot.__init__(self)

    # Speed preference (radians per second, converts to ~)
    speed_pref = 5
    # Movement precision preference (in meters)
    precision_pref = 0.005

    # Basal Sensor Readings
    initial_fle = 0
    initial_fre = 0

    # ERROR: No attribute "device" - tedium, activate
    # You should insert a getDevice-like function in order to get the
    # instance of a device of the robot. Something like:
    #  motor = robot.getDevice('motorname')
    #  ds = robot.getDevice('dsname')
    #  ds.enable(timestep)

    # Advance time (returns -1 when simulator calls for stop)
    def advance(self):
        if self.step(int(self.getBasicTimeStep())) == -1:
            exit(0)

    # Get relative total distance traveled based on initial encoder readings
    def relative_fle(self):
        return self.get_front_left_motor_encoder_reading() - self.initial_fre

    def relative_fre(self):
        return self.get_front_right_motor_encoder_reading() - self.initial_fre

    # Move forward a given distance (in m), calculated via sensor readings, NOT TIME
    def move_linear(self, distance):
        current_ae = (self.relative_fre() + self.relative_fle()) / 2
        # Move forward until average encoder reading is within acceptable
        # range of target distance
        self.set_right_motors_velocity(self.speed_pref)
        self.set_left_motors_velocity(self.speed_pref)
        while (self.relative_fre() + self.relative_fle() - current_ae) / 2 * self.wheel_radius <= distance - self.precision_pref:
            self.advance()
        self.stop()

    # Move in an arc a given distance with a certain radius - clock direction
    # will be determined by sign of radius
    def move_arc(self, distance, radius):
        return

    # Rotate inplace by a specified angle (in degrees):
    # + clockwise/- counterclockwise
    def rotate(self, angle):
        initial_angle = self.get_compass_reading()
        target_angle = initial_angle + angle
        print(target_angle)
        # Rotate inplace until within acceptable range of target angle
        while 1:
            if angle > 0 and self.get_compass_reading() <= target_angle - self.precision_pref:
                self.set_right_motors_velocity(self.speed_pref)
                self.set_left_motors_velocity(-self.speed_pref)
            elif angle < 0 and self.get_compass_reading() >= target_angle + self.precision_pref:
                self.set_right_motors_velocity(-self.speed_pref)
                self.set_left_motors_velocity(self.speed_pref)
            else:
                break
            self.advance()
        self.stop()
