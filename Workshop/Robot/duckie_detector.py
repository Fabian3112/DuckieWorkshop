#import cv2
#import numpy as np
#import torch

class DuckieDetector:
    def __init__(self, crop_bounds):
        
        #print(f'can use gpu: {torch.cuda.is_available()}')
        #self.model = YOLO("/workspace/Workshop/Robot/Model/Small_hue.pt")

        #self.top_left_x = crop_bounds['top_left_x']
        #self.top_left_y = crop_bounds['top_left_y']
        #self.top_right_x = crop_bounds['top_right_x']
        #self.top_right_y = crop_bounds['top_right_y']
        #self.bottom_left_x = crop_bounds['bottom_left_x']
        #self.bottom_left_y = crop_bounds['bottom_left_y']
        #self.bottom_right_x = crop_bounds['bottom_right_x']
        #self.bottom_right_y = crop_bounds['bottom_right_y']
        pass

    def detect_duckies(self, image):
        # Implement the duckie detection logic here
        # For now, return an empty list as a placeholder
        #min_x = np.min([self.top_left_x, self.top_right_x, self.bottom_left_x, self.bottom_right_x])
        #max_x = np.max([self.top_left_x, self.top_right_x, self.bottom_left_x, self.bottom_right_x])
        #min_y = np.min([self.top_left_y, self.top_right_y, self.bottom_left_y, self.bottom_right_y])
        #max_y = np.max([self.top_left_y, self.top_right_y, self.bottom_left_y, self.bottom_right_y])

        #results = self.model(image, conf=0.2, verbose=False)

        #for r in results:
        #    for box in r.boxes:
        #        print('detected duckie')
        #        x1, y1, x2, y2 = box.xyxy[0].tolist()

                # ground contact point = bottom-centre of the box
        #        cx = (x1 + x2) / 2.0
        #        cy = y2
                              
         #       cv2.circle(img=image,center=(int((x1+x2)/2) , int(len(y1+y2)/2)),radius=20,color=(0,0,255),thickness=5)
        
        
        return []