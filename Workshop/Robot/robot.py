import os

import numpy as np
from Robot.detect_lane import LaneDetector
from Robot.pid_controller import PIDController
from Robot.duckie_detector import DuckieDetector
from duckietown_msgs.msg import Twist2DStamped

import rospy

from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge



class Robot:
    def __init__(self):
        print("starting initialization")

        rospy.init_node('robot')

        self.image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.v = 0.0
        self.omega = 0.0

        #for image decompression
        self._bridge = CvBridge()

        vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = '/camera' #f"/{vehicle_name}/camera_node/image/compressed"
        twist_topic  = '/cmd_vel' #f"/{vehicle_name}/car_cmd_switch_node/cmd"

        #image subscriber
        self.image_listener = rospy.Subscriber(self._camera_topic, CompressedImage, self.image_callback)

        #vlelocity publisher
        self.velocity_publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=10)


        lane_detector = LaneDetector()
        self.detect_lane = lane_detector.calculate_lane_center
        pid_controller = PIDController()
        self.pid_controller = pid_controller.compute_control
    
        duckie_detector = DuckieDetector()
        self.detect_duckies = duckie_detector.detect_duckies

        print("done robot is ready")

    def image_callback(self, image_msg):
        image = self._bridge.compressed_imgmsg_to_cv2(image_msg)

        self.follow_lane(image)


    def follow_lane(self, image):
        print("starting lane following")

        lane_center, is_crossway = self.detect_lane(image)
        detected_duckies = self.detect_duckies(image)

        if is_crossway:
            self.v = 0.0
            self.omega = 0.0

        elif detected_duckies.size > 0:
            self.v = 0.0
            self.omega = 0.0

        else:
            self.v, self.omega = self.pid_controller(lane_center)

        self.publish_velocity(self.v, self.omega)

    def publish_velocity(self, v, omega):
        message = Twist2DStamped(v=v, omega=omega)
        self.velocity_publisher.publish(message)



