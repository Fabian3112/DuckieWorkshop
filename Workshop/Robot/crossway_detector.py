import cv2
import numpy as np
import rospy

class CrosswayDetector:
    def __init__(self):

        self.hue_red_l = 0
        self.hue_red_h = 10
        self.saturation_red_l = 100
        self.saturation_red_h = 255
        self.lightness_red_l = 100
        self.lightness_red_h = 255
        self.stop_line_threshold = 50

        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])

        self.lower_red2 = np.array([170, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])


        self.integral = 0.0
        self.previous_error = 0.0
        self.start_factor = 1
        self.Kp = 1.0
        self.Kd = 1.0
        self.Ki = 0.0
        pass

    def get_lines(self,mask):
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
        
        #print(f"Number of red lines: {len(lines)}")
        return lines



    def get_orientation(self,mask):
        #plt.imshow(mask)
        #plt.show()
        #print(f'calculating heading')
        lines = self.get_lines(mask)
        #print(f'got lines')

        error = 0
        if len(lines) == 0:
            #print(f'no lines')
            return error
            
        for line in lines:
                #print(f'line {line}')
                
                error = line["angle"] % (np.pi / 2)
                #print(f"Line center: {line['center']}, angle: {line['angle']}, error: {error}")

        #print(f'done heading calculation ')
        #print(f'returning {(np.degrees(error) + 45 ) % 90 -45}')
        return (np.degrees(error) + 45 ) % 90 -45


    def detect_crossway(self,image):
        #print(f'detecting crossway')
        croped_image = image[-150:,200:400]
        #print(f'croped image')
        
        mask = cv2.inRange(croped_image, self.lower_red1, self.upper_red1)
        mask |= cv2.inRange(croped_image, self.lower_red2, self.upper_red2)
        
        #print(f'generated mask')
    
        stop_line = np.count_nonzero(mask) >= 4_000
        
        #print(f'stopline {stop_line}')

        if stop_line:
            #print(f'found stopline')
            heading_errror = self.get_orientation(mask)
            #print(f'heading {heading_errror}')

            #print(f'where to go:')
            mask_full_image = cv2.inRange(image, self.lower_red1, self.upper_red1)
            mask_full_image |= cv2.inRange(image, self.lower_red2, self.upper_red2)

            #print(f'calculating exits: {mask_full_image.shape}')
            left_side = mask_full_image[200:300,:100]
            #print(f'left {left_side.shape}')
            left = np.count_nonzero(left_side) >= 10
            #print(f'left {left}')
            right_side = mask_full_image[100:250,400:500]
            right = np.count_nonzero(right_side) >= 10
            front_side = mask_full_image[100:200,100:300]
            straight = np.count_nonzero(front_side) >= 10

            
            #print(f'crosswadetection done')
            #print(f' {stop_line}, {heading_errror}, left: {left}, forward: {forward}, right: {right}')
            return stop_line, heading_errror, {'left': left, 'straight': straight, 'right': right}
        
        
        
        #if mask_red.count_nonzero() > self.stop_line_threshold:
        #    print("Red line detected")
        #    return True,0

        #print(f'crosswadetection done {False} {0} {0}')        
        return False,0, {'left': False, 'straight': False, 'right': False}

    def compute_pid(self, error):
        self.integral += error
        self.integral = max(-5, min(5, self.integral))
        derivative = error - self.previous_error

        if(error >= self.previous_error - 0.001):
            print(f'start_factor {self.start_factor}')
            self.start_factor += 0.5
        else:
            self.start_factor = 0
        
        omega = -((self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)) - math.copysign(self.start_factor,error)
        v = 0

        self.previous_error = error
        return (v, omega)