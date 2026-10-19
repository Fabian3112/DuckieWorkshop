import json

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class DuckieDetector:
    """Find red rubber ducks in the camera image by their colour (HSV)."""

    def __init__(self, crop_bounds=None, vehicle_name=None):
        # Red sits at both ends of the hue circle (OpenCV hue goes 0..180),
        # so there are two hue ranges, like for the red stop line.
        self.hue_1_l = 0
        self.hue_1_h = 10
        self.hue_2_l = 170
        self.hue_2_h = 180
        self.saturation_l = 120
        self.saturation_h = 255
        self.lightness_l = 80
        self.lightness_h = 255

        # A blob counts as a duck when it is big enough
        self.min_area = 1500        # pixels in the half size image (320 x 240)

        # Only look in front of the robot: middle of the image, not the sky
        self.roi_x_low = 0.2
        self.roi_x_high = 0.8
        self.roi_y_low = 0.35
        self.roi_y_high = 1.0

        # True: the robot drives around detected ducks, False: only detect (for tuning)
        self.evade = True

        self.scale = 0.5

        if vehicle_name is not None:
            self.settings_sub = rospy.Subscriber(f'/{vehicle_name}/duck_detection_node/hsv', String, self.settings_callback)

            debug_image_topic = f"/{vehicle_name}/robot/duck_image/compressed"
            self.debug_image_publisher = rospy.Publisher(debug_image_topic, CompressedImage, queue_size=1)
        else:
            self.debug_image_publisher = None

    def settings_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.hue_1_l = int(data["h1_low"])
            self.hue_1_h = int(data["h1_high"])
            self.hue_2_l = int(data["h2_low"])
            self.hue_2_h = int(data["h2_high"])
            self.saturation_l = int(data["s_low"])
            self.saturation_h = int(data["s_high"])
            self.lightness_l = int(data["v_low"])
            self.lightness_h = int(data["v_high"])
            if "min_area" in data:
                self.min_area = int(data["min_area"])
            if "evade" in data:
                self.evade = bool(data["evade"])
        except Exception as exc:
            rospy.logwarn_throttle(2.0, f"Failed parsing duck HSV JSON: {exc}")

    def get_mask(self, hsv_image):
        mask = cv2.inRange(hsv_image,
                           (self.hue_1_l, self.saturation_l, self.lightness_l),
                           (self.hue_1_h, self.saturation_h, self.lightness_h))
        mask |= cv2.inRange(hsv_image,
                            (self.hue_2_l, self.saturation_l, self.lightness_l),
                            (self.hue_2_h, self.saturation_h, self.lightness_h))
        return mask

    def detect_duckies(self, image):
        """Return a list of ducks as (x, y, width, height) boxes in the full camera image.

        `image` is the BGR camera image.
        """
        small = cv2.resize(image, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        mask = self.get_mask(hsv)
        # remove single noisy pixels
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        height, width = mask.shape
        roi_x1 = int(width * self.roi_x_low)
        roi_x2 = int(width * self.roi_x_high)
        roi_y1 = int(height * self.roi_y_low)
        roi_y2 = int(height * self.roi_y_high)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)

        duckies = []
        candidates = []
        for i in range(1, num_labels):
            x, y, w, h, area = stats[i]
            if area < 50:
                continue

            cx, cy = centroids[i]
            in_front = roi_x1 <= cx <= roi_x2 and roi_y1 <= cy <= roi_y2
            is_duck = in_front and area >= self.min_area

            candidates.append((x, y, w, h, area, is_duck))
            if is_duck:
                duckies.append((int(x / self.scale), int(y / self.scale),
                                int(w / self.scale), int(h / self.scale)))

        self.publish_debug_image(small, mask, (roi_x1, roi_y1, roi_x2, roi_y2), candidates, len(duckies) > 0)
        return duckies

    def publish_debug_image(self, image, mask, roi, candidates, found):
        # Only draw the image if someone (the frontend) is watching, this saves CPU
        if self.debug_image_publisher is None or self.debug_image_publisher.get_num_connections() == 0:
            return

        overlay = image.copy()
        overlay[mask > 0] = (255, 0, 255)
        view = cv2.addWeighted(image, 0.5, overlay, 0.5, 0)

        cv2.rectangle(view, (roi[0], roi[1]), (roi[2], roi[3]), (255, 255, 0), 1)
        for x, y, w, h, area, is_duck in candidates:
            color = (0, 255, 0) if is_duck else (160, 160, 160)
            cv2.rectangle(view, (x, y), (x + w, y + h), color, 2)
            cv2.putText(view, f'{area}', (x, max(10, y - 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)

        text = 'DUCK!' if found else 'no duck'
        cv2.putText(view, text, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if found else (255, 255, 255), 2)

        mask_view = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined = np.hstack((view, mask_view))

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', combined)[1]).tobytes()
        self.debug_image_publisher.publish(msg)
