import json
import rospy
from std_msgs.msg import String


class PIDController:

    def __init__(self, Kp=6, Ki=0.0, Kd=4, vehicle_name=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.MAX_VEL = 0.2

        self.integral = 0.0
        self.previous_error = 0.0

        self.pid_sub = None
        if vehicle_name:
            self.pid_sub = rospy.Subscriber(
                f'/{vehicle_name}/lane_control_node/pid',
                String,
                self.pid_callback,
                queue_size=1,
            )

    def pid_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.Kp = float(data["p"])
            self.Ki = float(data["i"])
            self.Kd = float(data["d"])
        except Exception as exc:
            rospy.logwarn_throttle(2.0, f"Failed parsing PID JSON: {exc}")


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