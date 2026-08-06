import rospy
import numpy as np
import cv2

class LaneDetector:
    def __init__(self):

        # init white line parameters
        self.hue_white_l = 0
        self.hue_white_h = 255
        self.saturation_white_l = 0
        self.saturation_white_h = 41
        self.lightness_white_l = 161
        self.lightness_white_h = 255
        
        # init yellow line parameters
        self.hue_yellow_l = 15
        self.hue_yellow_h = 60
        self.saturation_yellow_l = 60
        self.saturation_yellow_h = 255
        self.lightness_yellow_l = 120
        self.lightness_yellow_h = 255

        
        self.distance_look_ahead = 0.75
        pass


    def get_x_for_driving(self,distance, mask, no_lane_value, left_line):
        grad = cv2.Sobel(mask, cv2.CV_16S, 1, 0, ksize=3, scale=1, delta=0, borderType=cv2.BORDER_DEFAULT)
        _,th1 = cv2.threshold(grad,127,255,cv2.THRESH_BINARY)

        a = []
        for row in range(distance-50, distance+50):
            if np.where(th1[row] == 255)[0].size == 0:
                continue
            else:
                if left_line:
                    a.append(np.where(th1[row] == 255)[0][-1])
                else:
                    a.append(np.where(th1[row] == 255)[0][0])

        if len(a) > 10:
            return np.median(a)
        else:
            return no_lane_value
    
    def calculate_lane_center(self,  image):

        mask_yellow = cv2.inRange(image, 
                           (self.hue_yellow_l,self.saturation_yellow_l, self.lightness_yellow_l), 
                           (self.hue_yellow_h,self.saturation_yellow_h, self.lightness_yellow_h),)
        
        mask_white = cv2.inRange(image, 
                           (self.hue_white_l,self.saturation_white_l, self.lightness_white_l), 
                           (self.hue_white_h,self.saturation_white_h, self.lightness_white_h),)
        
        white_alternative = int(len(image[0]) * 0.95)
        yellow_alternative = int(len(image[0]) * 0.05)

        center_white = self.get_x_for_driving(mask_white,int(len(image)*self.distance_look_ahead), white_alternative, left_line=False)
        center_yellow = self.get_x_for_driving(mask_yellow,int(len(image)*self.distance_look_ahead), yellow_alternative, left_line=True)


        if center_white <= center_yellow:
            if center_white > int(len(image[0]) * 0.4):
                center_yellow = yellow_alternative
            else:
                center_white = white_alternative
        
        lane_center = (center_white + center_yellow) / 2


        return lane_center