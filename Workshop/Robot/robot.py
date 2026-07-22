import numpy as np
import detect_lane
import rospy


class Robot:
    def __init__(self):
        print("starting initialization")

        self.image = np.zeros((480, 640, 3), dtype=np.uint8)
        self. v = 0.0
        self.omega = 0.0


        print("done robot is ready")

    def start(self):
        print("starting lane following")



