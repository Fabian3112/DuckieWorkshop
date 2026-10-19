import json

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class DuckieDetector:
    """Find rubber ducks in the cropped camera image by their colour (HSV)."""

    def __init__(self, vehicle_name=None):
        # OpenCV hue goes from 0 to 180. If hue_l is bigger than hue_h the range
        # wraps around: everything above hue_l or below hue_h counts.
        # Red sits at both ends of the hue circle, so the default wraps.
        self.hue_l = 170
        self.hue_h = 10
        self.saturation_l = 120
        self.saturation_h = 255
        self.lightness_l = 80
        self.lightness_h = 255

        # A blob counts as a duck when it has at least this many pixels
        self.min_area = 300

        # True: the robot drives around detected ducks, False: it stops in front of them
        self.evade = True

        if vehicle_name is not None:
            self.settings_sub = rospy.Subscriber(f'/{vehicle_name}/duck_detection_node/hsv', String, self.settings_callback)

            debug_image_topic = f"/{vehicle_name}/robot/duck_image/compressed"
            self.debug_image_publisher = rospy.Publisher(debug_image_topic, CompressedImage, queue_size=1)
        else:
            self.debug_image_publisher = None

    def settings_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.hue_l = int(data["h_low"])
            self.hue_h = int(data["h_high"])
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
        if self.hue_l <= self.hue_h:
            return cv2.inRange(hsv_image,
                               (self.hue_l, self.saturation_l, self.lightness_l),
                               (self.hue_h, self.saturation_h, self.lightness_h))

        # wrapped hue range: from hue_l up to 180 and from 0 up to hue_h
        mask = cv2.inRange(hsv_image,
                           (self.hue_l, self.saturation_l, self.lightness_l),
                           (180, self.saturation_h, self.lightness_h))
        mask |= cv2.inRange(hsv_image,
                            (0, self.saturation_l, self.lightness_l),
                            (self.hue_h, self.saturation_h, self.lightness_h))
        return mask

    def detect_duckies(self, hsv_image, image):
        """Return a list of ducks as (x, y, width, height) boxes.

        `hsv_image` is the cropped HSV image of the robot, `image` the same crop in BGR.
        """
        mask = self.get_mask(hsv_image)
        # remove single noisy pixels
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)

        duckies = []
        candidates = []
        for i in range(1, num_labels):
            x, y, w, h, area = stats[i]
            if area < 20:
                continue

            is_duck = area >= self.min_area
            candidates.append((x, y, w, h, area, is_duck))
            if is_duck:
                duckies.append((int(x), int(y), int(w), int(h)))

        self.publish_debug_image(image, mask, candidates, len(duckies) > 0)
        return duckies

    def publish_debug_image(self, image, mask, candidates, found):
        # Only draw the image if someone (the frontend) is watching, this saves CPU
        if self.debug_image_publisher is None or self.debug_image_publisher.get_num_connections() == 0:
            return

        overlay = image.copy()
        overlay[mask > 0] = (255, 0, 255)
        view = cv2.addWeighted(image, 0.5, overlay, 0.5, 0)

        for x, y, w, h, area, is_duck in candidates:
            color = (0, 255, 0) if is_duck else (160, 160, 160)
            cv2.rectangle(view, (x, y), (x + w, y + h), color, 2)
            cv2.putText(view, f'{area}', (x, max(10, y - 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)

        text = 'DUCK!' if found else 'no duck'
        cv2.putText(view, text, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0) if found else (255, 255, 255), 1)

        mask_view = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined = np.hstack((view, mask_view))

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', combined)[1]).tobytes()
        self.debug_image_publisher.publish(msg)
