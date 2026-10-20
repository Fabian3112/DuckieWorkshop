import json

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class DuckieDetector:
    """Find rubber ducks in the cropped camera image by their colour (HSV)."""

    def __init__(self, vehicle_name=None):
        # Ducks can have different colours. A pixel counts if it matches any of them.
        # OpenCV hue goes from 0 to 180. If h_low is bigger than h_high the range
        # wraps around: everything above h_low or below h_high counts.
        # Red sits at both ends of the hue circle, so the default wraps.
        self.colours = [
            {'h_low': 170, 'h_high': 10, 's_low': 120, 's_high': 255, 'v_low': 80, 'v_high': 255},
        ]

        # A blob counts as a duck when it has at least this many pixels
        self.min_area = 300

        # Only react when the duck is close: its lowest pixel must be below this row
        # of the cropped image (0 = top/far away, 200 = bottom/right in front of the robot)
        self.trigger_line = 150

        # True: the robot drives around detected ducks, False: it ignores them
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
            keys = ['h_low', 'h_high', 's_low', 's_high', 'v_low', 'v_high']
            self.colours = [{key: int(colour[key]) for key in keys} for colour in data["colours"]]
            if "min_area" in data:
                self.min_area = int(data["min_area"])
            if "trigger_line" in data:
                self.trigger_line = int(data["trigger_line"])
            if "evade" in data:
                self.evade = bool(data["evade"])
        except Exception as exc:
            rospy.logwarn_throttle(2.0, f"Failed parsing duck HSV JSON: {exc}")

    def get_colour_mask(self, hsv_image, colour):
        low = (colour['h_low'], colour['s_low'], colour['v_low'])
        high = (colour['h_high'], colour['s_high'], colour['v_high'])
        if colour['h_low'] <= colour['h_high']:
            return cv2.inRange(hsv_image, low, high)

        # wrapped hue range: from h_low up to 180 and from 0 up to h_high
        mask = cv2.inRange(hsv_image, low, (180, high[1], high[2]))
        mask |= cv2.inRange(hsv_image, (0, low[1], low[2]), high)
        return mask

    def get_mask(self, hsv_image):
        mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        for colour in self.colours:
            mask |= self.get_colour_mask(hsv_image, colour)
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

            is_duck = area >= self.min_area and y + h >= self.trigger_line
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

        width = view.shape[1]
        cv2.line(view, (0, self.trigger_line), (width, self.trigger_line), (0, 200, 255), 1)

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
