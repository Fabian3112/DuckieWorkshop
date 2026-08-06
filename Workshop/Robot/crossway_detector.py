import cv2

class CrosswayDetector:
    def __init__(self):

        self.hue_red_l = 0
        self.hue_red_h = 10
        self.saturation_red_l = 100
        self.saturation_red_h = 255
        self.lightness_red_l = 100
        self.lightness_red_h = 255
        self.stop_line_threshold = 50
        pass

    def detect_crossway(self,croped_image,image):

        mask_red = cv2.inRange(croped_image, 
                           (self.hue_red_l,self.saturation_red_l, self.lightness_red_l), 
                           (self.hue_red_h,self.saturation_red_h, self.lightness_red_h),)

        if mask_red.count_nonzero() > self.stop_line_threshold:
            print("Red line detected")
            return True,0
        
        return False,0  