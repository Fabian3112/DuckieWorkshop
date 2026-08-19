import cv2
import numpy as np
from enum import Enum

class CrosswayType(Enum):
    LeftStraightRight = 1
    LeftStraight = 2
    StraightRight = 3
    LeftRight = 4

    
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

    def crop_image_crossway(self,image):

        pts1 = np.float32([
            [0,     0],
            [500,    0],
            [0, 500],
            [500,  500],])
        
        pts2 = np.float32([[0,0],[400,0],[0,400],[400,400]])

        M = cv2.getPerspectiveTransform(pts1,pts2)
        return cv2.warpPerspective(image,M,(400,400))

    def get_lines(self, mask):
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)


        lines = []
        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]

            if area < 100:
                continue

            ys, xs = np.where(labels == i)

            points = np.column_stack((xs, ys)).astype(np.float32)

            vx, vy, x0, y0 = cv2.fitLine(
                points,
                cv2.DIST_L2,
                0,
                0.01,
                0.01
            )

            angle = np.arctan2(float(vy), float(vx))

            lines.append({
                "center": (float(x0), float(y0)),
                "angle": angle,
                "area": area
            })
        
        print(f"Number of red lines: {len(lines)}")
        return lines
        


    def detect_crossway(self,croped_image,image):

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask = cv2.inRange(croped_image, lower_red1, upper_red1)
        mask |= cv2.inRange(croped_image, lower_red2, upper_red2)
        if cv2.countNonZero(mask) > self.stop_line_threshold:
            print("Red line detected")

            #Find all the corssway exits
            image = self.crop_image_crossway(image)
            mask_full_image = cv2.inRange(image, lower_red1, upper_red1)
            mask_full_image |= cv2.inRange(image, lower_red2, upper_red2)

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            lines = self.get_lines(mask_full_image)

            heading_error = 0.0
            for line in lines:
                
                error = line["angle"] % (np.pi / 2)
                print(f"Line center: {line['center']}, angle: {line['angle']}, error: {error}")
                heading_error += error/len(lines)

            print(f"Average heading error: {heading_error} \n ----------------- \n\n")
            return True,heading_error, len(lines)

        
        return False,0, 0