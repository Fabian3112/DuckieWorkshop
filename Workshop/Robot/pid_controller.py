class PIDController:

    def __init__(self, Kp=0.5, Ki=0.1, Kd=0.05):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.MAX_VEL = 0.2

        self.integral = 0.0
        self.previous_error = 0.0


    # lane center is normalized between -1 and 1, where 0 is the center of the lane
    def compute_control(self, lane_center):
        error = lane_center
        self.integral += error
        derivative = error - self.previous_error

        omega = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        v = min(0.1, self.MAX_VEL - abs(omega))

        self.previous_error = error

        return omega