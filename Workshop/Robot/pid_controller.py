class PIDController:

    def __init__(self, Kp=9, Ki=0.0, Kd=5):
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
        self.integral = max(5,self.integral)
        derivative = error - self.previous_error

        omega = -((self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative))
        v = max(0.05, self.MAX_VEL - abs(omega)*0.1)

        self.previous_error = error
        #print(f'error was : {error}')
        #print(f'driving with v={v}, omega={omega}')

        return v, omega