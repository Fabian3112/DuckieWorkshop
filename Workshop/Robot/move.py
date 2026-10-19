import rospy
import math
import time
from duckietown_msgs.msg import WheelEncoderStamped
from duckietown_msgs.msg import Twist2DStamped

class Mover:
    """Track Duckiebot pose from wheel encoders and build a coarse tile map."""

    def __init__(
        self,
        vehicle_name: str,
        wheel_radius: float = 0.0318,
        ticks_per_revolution: int = 135,
        baseline: float = 0.1,
    ):
        self.wheel_radius = float(wheel_radius)
        self.ticks_per_revolution = int(ticks_per_revolution)
        self.baseline = float(baseline)

        self._left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self._left_tick = None
        self._right_tick = None
        self._prev_left_tick = None
        self._prev_right_tick = None

        self.sub_left = rospy.Subscriber(
            self._left_encoder_topic,
            WheelEncoderStamped,
            self.callback_left,
            queue_size=1,
        )
        self.sub_right = rospy.Subscriber(
            self._right_encoder_topic,
            WheelEncoderStamped,
            self.callback_right,
            queue_size=1,
        )

        #vlelocity publisher
        twist_topic  = f"/{vehicle_name}/car_cmd_switch_node/cmd"
        self.velocity_publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

    def callback_left(self, msg: WheelEncoderStamped):
        self._left_tick = int(msg.data)
        self._update_position()

    def callback_right(self, msg: WheelEncoderStamped):
        self._right_tick = int(msg.data)
        self._update_position()

    def _update_position(self):
        if self._left_tick is None or self._right_tick is None:
            return

        if self._prev_left_tick is None or self._prev_right_tick is None:
            self._prev_left_tick = self._left_tick
            self._prev_right_tick = self._right_tick
            return

        delta_left_ticks = self._left_tick - self._prev_left_tick
        delta_right_ticks = self._right_tick - self._prev_right_tick

        if delta_left_ticks == 0 and delta_right_ticks == 0:
            return


        delta_phi_left = (2.0 * math.pi * delta_left_ticks) / self.ticks_per_revolution
        delta_phi_right = (2.0 * math.pi * delta_right_ticks) / self.ticks_per_revolution

        d_left = self.wheel_radius * delta_phi_left
        d_right = self.wheel_radius * delta_phi_right
        d_center = 0.5 * (d_left + d_right)
        delta_theta = (d_right - d_left) / self.baseline

        theta_mid = self.theta + 0.5 * delta_theta
        self.x += d_center * math.cos(theta_mid)
        self.y += d_center * math.sin(theta_mid)
        self.theta = self._wrap_angle(self.theta + delta_theta)

        self._prev_left_tick = self._left_tick
        self._prev_right_tick = self._right_tick

    def forward(self, distance: float):

        start_x = self.x
        start_y = self.y
        theta_start = self.theta
        
        error = distance
        previous_error = error
        integral_error = 0.0
        print(f'starting to move')
        while abs(error) > 0.01 and not rospy.is_shutdown():
            error = distance - math.hypot(self.x - start_x, self.y - start_y)

            kp = 1.0
            kd = 0.5
            ki = 0.1

            max_v = 0.2
            derivative_error = error - previous_error
            integral_error += error

            v = kp * error + kd * derivative_error + ki * integral_error
            v = min(v, max_v)

            
            omega = theta_start - self.theta
            self.publish_velocity(v, omega)

            previous_error = error
            time.sleep(0.05)

        
        print(f'done moving')
        self.publish_velocity(0.0, 0.0)  # stop the robot after reaching the goal
        time.sleep(0.1)
        self.publish_velocity(0.0, 0.0)  # stop the robot after reaching the goal
        
    def turn(self, angle: float):
        angle_rad = math.radians(angle)
        self.goal_theta = self._wrap_angle(self.theta + angle_rad)

        error_theta = self._wrap_angle(self.goal_theta - self.theta)
        previous_error = error_theta
        integral_error = 0.0

        print(f'starting error was {math.degrees(error_theta)}° \n\n')

        correct_count = 0

        while abs(error_theta) > math.radians(2.0) and not rospy.is_shutdown():

            if abs(error_theta) < math.radians(2.0):
                correct_count += 1
            else:
                correct_count = 0
                
            time.sleep(0.05)
            kp = 2.0
            kd = 10.0
            ki = 0.3

            max_omega = 10

            if abs(error_theta) >= abs(previous_error) - 0.001: 
                integral_error += error_theta
            
            derivative_error = error_theta - previous_error
            max_integral = max_omega / ki / 2
            integral_error = max(-max_integral, min(integral_error, max_integral))
            omega = kp * error_theta + kd * derivative_error + ki * integral_error
            omega = max(-max_omega, min(omega, max_omega))

            v = 0.0
            self.publish_velocity(v, omega)

            previous_error = error_theta
            error_theta = self._wrap_angle(self.goal_theta - self.theta)
  
            #print(f'new error is {math.degrees(error_theta)}° omega is {omega}')
            

        self.publish_velocity(0.0, 0.0)  # stop the robot after reaching the goal
        time.sleep(0.1)
        self.publish_velocity(0.0, 0.0)  # stop the robot after reaching the goal
        time.sleep(3)
        print(f'turn error after: {math.degrees(self._wrap_angle(self.goal_theta - self.theta))}')

    @staticmethod
    def _wrap_angle(angle_rad: float) -> float:
        return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi

    def publish_velocity(self, v: float, omega: float):
        twist_msg = Twist2DStamped()
        twist_msg.v = v
        twist_msg.omega = omega
        self.velocity_publisher.publish(twist_msg)
        


    
        
        