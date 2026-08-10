import os

import numpy as np
from Robot.detect_lane import LaneDetector
from Robot.pid_controller import PIDController
from Robot.duckie_detector import DuckieDetector
from Robot.crossway_detector import CrosswayDetector
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Int32


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



        lane_detector = LaneDetector()
        self.detect_lane = lane_detector.calculate_lane_center
        pid_controller = PIDController()
        self.pid_controller = pid_controller.compute_control
    
        duckie_detector = DuckieDetector()
        self.detect_duckies = duckie_detector.detect_duckies

        crossway_detector = CrosswayDetector()
        self.detect_crossway = crossway_detector.detect_crossway

        self.state = State.Driving


        self._crop_im_size = 200
        self.top_left_x = 159
        self.top_left_y = 218
        self.top_right_x = 441
        self.top_right_y = 218
        self.bottom_left_x = 606
        self.bottom_left_y = 382
        self.bottom_right_x = -29
        self.bottom_right_y = 382
        self.is_running = False
        self.method_blocked = False
        
        print("initialising ros publishers")
        vehicle_name = os.environ['HOSTNAME']
        
        #vlelocity publisher
        twist_topic  = f"/{vehicle_name}/car_cmd_switch_node/cmd"
        self.velocity_publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

        #debug image publisher
        debug_image_topic = f"/{vehicle_name}/robot/image/compressed"
        self.debug_image_publisher = rospy.Publisher(debug_image_topic, CompressedImage, queue_size=1)

        print("Initialising ros subscriber")
        #image subscriber
        self._camera_topic = f"/{vehicle_name}/camera_node/image/compressed"
        self.image_listener = rospy.Subscriber(self._camera_topic, CompressedImage, self.image_callback)

        #start robot subscriber
        start_topic = f"/{vehicle_name}/start"
        self.start_subscriber = rospy.Subscriber(start_topic, Int32, self.callback_start)
        
        
        print("done robot is ready")

    def start(self):
        self.is_running = True
    def stop(self):
        self.is_running = False
        self.publish_velocity(0.0,0.0)

    def callback_start(self,msg):
        if msg.data == 1:
            self.start()
        else:
            self.stop()

    
    def image_callback(self, image_msg):
        image = self._bridge.compressed_imgmsg_to_cv2(image_msg)

        if self.is_running:
            if not self.method_blocked:
                try:
                    self.method_blocked = True
                    self.follow_lane(image)
                    self.method_blocked = False
                except:
                    self.method_blocked = False
        


    def follow_lane(self, image):
        #print("starting lane following")
        
        detected_duckies = self.detect_duckies(image)

        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        croped_image = self.crop_image(hsv_img)
        debug_img = self.crop_image(image)

        #print(f'original image {hsv_img.shape} croped {croped_image.shape}')

        is_crossway, crossway_heading_error, crossway_type = self.detect_crossway(croped_image,hsv_img)

        if not is_crossway:
            lane_center = self.detect_lane(croped_image,debug_img)

        if is_crossway:
            self.v = 0.0
            self.omega = 0.0

        elif len(detected_duckies) > 0:
            self.v = 0.0
            self.omega = 0.0

        else:
            self.v, self.omega = self.pid_controller(lane_center)

        self.publish_velocity(self.v, self.omega)
        self.publish_image(debug_img)
        #print("finished lane following")

    def publish_velocity(self, v, omega):
        message = Twist2DStamped(v=v, omega=omega)
        self.velocity_publisher.publish(message)

    def publish_image(self,image):
        debug_msg = CompressedImage()
        debug_msg.header.stamp = rospy.Time.now()
        debug_msg.format = "jpeg"
        debug_msg.data = np.array(cv2.imencode('.jpg', image)[1]).tobytes()
        self.debug_image_publisher.publish(debug_msg)


    def crop_image(self,img):
                
        pts1 = np.float32([
            [self.top_left_x,     self.top_left_y],
            [self.top_right_x,    self.top_right_y],
            [self.bottom_right_x, self.bottom_right_y],
            [self.bottom_left_x,  self.bottom_left_y],])
        
        pts2 = np.float32([[0,0],[self._crop_im_size,0],[0,self._crop_im_size],[self._crop_im_size,self._crop_im_size]])

        M = cv2.getPerspectiveTransform(pts1,pts2)
        return cv2.warpPerspective(img,M,(self._crop_im_size,self._crop_im_size))
