import json
import math
import os
import random
import time
from typing import Dict, Optional, Tuple

from attr import dataclass
import cv2
import numpy as np
from enum import Enum
import rospy
from duckietown_msgs.msg import WheelEncoderStamped
import threading
from Robot.tracking.ParticleFilter import ParticleFilter
from Robot.tracking.LaneSegment import LaneSegment, TileType
from sensor_msgs.msg import CompressedImage


class Tracker:

    class Tile:
        def __init__(self, tile_type: 'Tracker.TileType', position: Tuple[int, int], orientation: float):
            self.tile_type = tile_type
            self.position = position  # (x, y) coordinates of the tile in the map
            self.orientation = orientation  # orientation of the tile in radians
    
    def __init__(
        self,
        vehicle_name: str,
        wheel_radius: float = 0.0318,
        ticks_per_revolution: int = 135,
        baseline: float = 0.1,
        segment_distance: float = 0.60,
        significant_turn_threshold_rad: float = math.radians(90.0),
        map_output_path: Optional[str] = None,
    ):
        self.wheel_radius = float(wheel_radius)
        self.ticks_per_revolution = int(ticks_per_revolution)
        self.baseline = float(baseline)
        self.segment_distance = float(segment_distance)
        self.significant_turn_threshold_rad = float(significant_turn_threshold_rad)

        self._left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

        self.x = 0.0
        self.y = 0.15
        self.theta = 0.0
        

        self._left_tick = None
        self._right_tick = None
        self._prev_left_tick = None
        self._prev_right_tick = None

        # Initialize the map as a dictionary to store tile information
        self.map: Dict[Tuple[int, int], Dict[str, any]] = {}
        self.particle_filter = ParticleFilter(n_particles=100)
        self.particle_filter.reset(self.x, self.y, self.theta)

        # Subscribe to wheel encoder topics
        rospy.Subscriber(
            self._left_encoder_topic,
            WheelEncoderStamped,
            self.callback_left,
            queue_size=1,
        )
        rospy.Subscriber(
            self._right_encoder_topic,
            WheelEncoderStamped,
            self.callback_right,
            queue_size=1,
        )

        # Publisher for map as image
        map_topic = f"/{vehicle_name}/map_image"
        self.map_publisher = rospy.Publisher(map_topic, CompressedImage, queue_size=1)

        threading.Thread(target=self.run, daemon=True).start()


    def callback_left(self, msg: WheelEncoderStamped):
        self._left_tick = int(msg.data)
        self._update_position()

    def callback_right(self, msg: WheelEncoderStamped):
        self._right_tick = int(msg.data)
        self._update_position()

    def _update_position(self):
        if self._left_tick is None or self._right_tick is None:
            return

        if self._prev_left_tick is None or self._prev_right_tick is None:
            self._prev_left_tick = self._left_tick
            self._prev_right_tick = self._right_tick
            return

        delta_left_ticks = self._left_tick - self._prev_left_tick
        delta_right_ticks = self._right_tick - self._prev_right_tick

        if delta_left_ticks == 0 and delta_right_ticks == 0:
            return


        delta_phi_left = (2.0 * math.pi * delta_left_ticks) / self.ticks_per_revolution
        delta_phi_right = (2.0 * math.pi * delta_right_ticks) / self.ticks_per_revolution

        d_left = self.wheel_radius * delta_phi_left
        d_right = self.wheel_radius * delta_phi_right
        d_center = 0.5 * (d_left + d_right)
        delta_theta = (d_right - d_left) / self.baseline

        theta_mid = self.theta + 0.5 * delta_theta
        self.x += d_center * math.cos(theta_mid)
        self.y += d_center * math.sin(theta_mid)
        self.theta = self._wrap_angle(self.theta + delta_theta)

        self._prev_left_tick = self._left_tick
        self._prev_right_tick = self._right_tick


    def draw_map(self):
        map, point = self.particle_filter.estimate_full_map()

        points = []
        for key in map:
            tile = map[key]
            points.extend(tile.points)

        map_img = LaneSegment.draw_points(points,point)

        return map_img


    def run(self):
        prev_x, prev_y, prev_theta = self.x, self.y, self.theta
        self.particle_filter.reset(self.x, self.y, self.theta)

        while not rospy.is_shutdown():
            time.sleep(0.1)
            
            distance = math.sqrt((self.x - prev_x) ** 2 + (self.y - prev_y) ** 2)
            rotation = self._wrap_angle(self.theta - prev_theta)

            self.particle_filter.predict(distance, rotation)

            prev_x, prev_y, prev_theta = self.x, self.y, self.theta

            
            self.particle_filter.calculate_weights()
            self.particle_filter.normalize()
            self.particle_filter.estimate()
            self.particle_filter.resample()

            map_img = self.draw_map()
            compressed_map_img = CompressedImage()
            compressed_map_img.header.stamp = rospy.Time.now()
            compressed_map_img.format = "jpeg"
            compressed_map_img.data = cv2.imencode('.jpg', map_img)[1].tobytes()
            self.map_publisher.publish(compressed_map_img)

    @staticmethod
    def _wrap_angle(angle_rad: float) -> float:
        return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi
            