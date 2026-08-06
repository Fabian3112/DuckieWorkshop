import os

import numpy as np
from Robot.detect_lane import LaneDetector
from Robot.pid_controller import PIDController
from Robot.duckie_detector import DuckieDetector
from Robot.crossway_detector import CrosswayDetector
from duckietown_msgs.msg import Twist2DStamped


import rospy

from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge


from enum import Enum

class State(Enum):
    Driving = 1
    Stop = 2
    Intersection = 3
    Duckie = 4


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
        self._camera_topic = f"/{vehicle_name}/camera_node/image/compressed"
        twist_topic  = f"/{vehicle_name}/car_cmd_switch_node/cmd"

        #image subscriber
        self.image_listener = rospy.Subscriber(self._camera_topic, CompressedImage, self.image_callback)

        #vlelocity publisher
        self.velocity_publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

        #debug image publisher

        debug_image_topic = f"/{vehicle_name}/robot/image/compressed"
        self.debug_image_publisher = rospy.Publisher(debug_image_topic, CompressedImage, queue_size=1)


        lane_detector = LaneDetector()
        self.detect_lane = lane_detector.calculate_lane_center
        pid_controller = PIDController()
        self.pid_controller = pid_controller.compute_control
    
        duckie_detector = DuckieDetector()
        self.detect_duckies = duckie_detector.detect_duckies

        crossway_detector = CrosswayDetector()
        self.detect_crossway = crossway_detector.detect_crossway

        self.state = State.Driving


        self._crop_im_size = 400
        self.top_left_x = 159
        self.top_left_y = 218
        self.top_right_x = 441
        self.top_right_y = 218
        self.bottom_left_x = 606
        self.bottom_left_y = 382
        self.bottom_right_x = -29
        self.bottom_right_y = 382
        self.distance_look_ahead = 0.75
        print("done robot is ready")

    def image_callback(self, image_msg):
        image = self._bridge.compressed_imgmsg_to_cv2(image_msg)

        self.follow_lane(image)


    def follow_lane(self, image):
        print("starting lane following")

        
        detected_duckies = self.detect_duckies(image)

        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        croped_image = self.crop_image(hsv_img)

        print(f'original image {hsv_img.shape} croped {croped_image.shape}')

        is_crossway, crossway_heading_error, crossway_type = self.detect_crossway(croped_image,hsv_img)

        if not is_crossway:
            lane_center = self.detect_lane(croped_image)

        if is_crossway:
            self.v = 0.0
            self.omega = 0.0

        elif detected_duckies.size > 0:
            self.v = 0.0
            self.omega = 0.0

        else:
            self.v, self.omega = self.pid_controller(lane_center)

        self.publish_velocity(self.v, self.omega)
        self.publish_image(image)

    def publish_velocity(self, v, omega):
        message = Twist2DStamped(v=v, omega=omega)
        self.velocity_publisher.publish(message)

    def publish_image(self,image):
        debug_msg = CompressedImage()
        debug_msg.header.stamp = rospy.Time.now()
        debug_msg.format = "jpeg"
        debug_msg.data = np.array(cv2.imencode('.jpg', image)[1]).tobytes()
        self.debug_image_publisher.publish(debug_msg)


    def crop_img(self,img):
                
        pts1 = np.float32([
            [self.top_left_x,     self.top_left_y],
            [self.top_right_x,    self.top_right_y],
            [self.bottom_right_x, self.bottom_right_y],
            [self.bottom_left_x,  self.bottom_left_y],])
        
        pts2 = np.float32([[0,0],[self._crop_im_size,0],[0,self._crop_im_size],[self._crop_im_size,self._crop_im_size]])

        M = cv2.getPerspectiveTransform(pts1,pts2)
        return cv2.warpPerspective(img,M,(self._crop_im_size,self._crop_im_size))

