import numpy as np
import math
from typing import Tuple
import cv2
from enum import Enum

class TileType(Enum):
        Straight = 1
        LeftTurn = 2
        RightTurn = 3
        #Intersection = 4


points_dict = {}


class LaneSegment:
    def __init__(self, map_position: Tuple[float, float], theta: float,type: TileType):

        #print(f"Creating LaneSegment at map position {map_position} with orientation {math.degrees(theta)} degrees and type {type.name}")
        #self.start_point = start_point
        self.orientation_quadrant = round(theta / (math.pi / 2)) % 4
        self.orientation = self.orientation_quadrant * (math.pi / 2)
        self.type = type
        self.map_position = map_position #(round(start_point[0]/0.6), round(start_point[1]/0.6))
        self.points = self._get_points_fast(type=self.type, orientation=self.orientation)

        #add points in the opposite direction of the orientation to the start of the points list, so that the lane segment can be driven in both directions
        if self.type == TileType.Straight:
            points_opposite = self._get_points_fast(type=TileType.Straight, orientation=(self.orientation + math.pi) % (2 * math.pi))
        if self.type == TileType.LeftTurn:
            points_opposite = self._get_points_fast(type=TileType.RightTurn, orientation=(self.orientation + math.pi /2 * 3) % (2 * math.pi))
        if self.type == TileType.RightTurn:
            points_opposite = self._get_points_fast(type=TileType.LeftTurn, orientation=(self.orientation + math.pi /2) % (2 * math.pi))

        self.points = points_opposite + self.points

    
    def get_end_point(self):
        return self.points[-1] 

    def _get_points_fast(self,type: TileType = None, orientation: float = None, map_position: Tuple[float, float] = (0, 0)):

        if type is None:
            type = self.type
        if orientation is None:
            orientation = self.orientation
        if map_position is None:
            map_position = self.map_position

        if (type, orientation) not in points_dict:
            print(f"Generating points for type {type.name} and orientation {math.degrees(orientation)} degrees")
            points_dict[(type, orientation)] = self._get_points(type=type, orientation=orientation, map_position=(0,0))

        points = points_dict[(type, orientation)].copy()

        offset_x, offset_y = self.map_position[0] * 0.6, self.map_position[1] * 0.6
        #rint(f'points before offset: {points}')
        points = [(x + offset_x, y + offset_y, theta) for (x, y, theta) in points]
        #print(f'points after offset: {points}')
        return points


    def _get_points(self, type: TileType = None, orientation: float = None, map_position: Tuple[float, float] = None):

        if type is None:
            type = self.type
        if orientation is None:
            orientation = self.orientation
        if map_position is None:
            map_position = self.map_position


        offset_x, offset_y = map_position[0] * 0.6, map_position[1] * 0.6

        if type == TileType.Straight:
            x = np.linspace(0, 0.6, 20)
            y = [0.15] * 20
            theta = [orientation] * 20

            x_rotated, y_rotated = self.rotate(orientation, x, y)

        if type == TileType.RightTurn:
            radius = 0.15
            angles = np.linspace(np.pi / 2, 0, 20)
            x = radius * np.cos(angles)
            y = radius * np.sin(angles)
            theta = (np.pi / 2) - angles

            x_rotated, y_rotated = self.rotate(orientation, x, y)

            theta = [(orientation + f) % (2 * math.pi) for f in theta]

        if type == TileType.LeftTurn:
            radius = 0.45
            center = (0, 0.6)
            angles = np.linspace(np.pi*2 , np.pi *3/2, 20)
            x = list(reversed(center[0] + radius * np.cos(angles)))
            y = list(reversed(center[1] + radius * np.sin(angles)))
            theta = angles
            x_rotated, y_rotated = self.rotate(orientation, x, y)

            theta = [(orientation + f) % (2 * math.pi) for f in theta]

        # Apply offset to all points
        x_rotated = [x + offset_x for x in x_rotated]
        y_rotated = [y + offset_y for y in y_rotated]


        if type == TileType.LeftTurn:
            start_theta = orientation
            step = math.pi / 2  / 20
            theta = [(start_theta + step * i) % (2 * math.pi) for i in range(20)]

        
        if type == TileType.RightTurn:
            start_theta = orientation
            step = math.pi / 2  / 20
            theta = [(start_theta - step * i) % (2 * math.pi) for i in range(20)]

        return list(zip(x_rotated, y_rotated, theta))

    def draw(self):
        image = np.zeros((60, 60, 3), dtype=np.uint8)
        for point in self.points:
            cv2.circle(image, (int(point[0]%60 * 100), int(point[1]%60 * 100)), 2, (0, 255, 0), -1)

        # draw small boarders around the tile
        cv2.rectangle(image, (0, 0), (59, 59), (255, 255, 255), 1)
        return image


    def next_tile_orientation(self):
        if self.type == TileType.Straight:
            return self.orientation
        elif self.type == TileType.LeftTurn:
            return (self.orientation + math.pi / 2) % (2 * math.pi)
        elif self.type == TileType.RightTurn:
            return (self.orientation - math.pi / 2) % (2 * math.pi)

    # offsets for facing East, South, West, North (quadrant 0-3), matching the Straight tile's own convention
    _DIRECTION_OFFSETS = {0: (1, 0), 1: (0, 1), 2: (-1, 0), 3: (0, -1)}

    def next_tile_map_position(self):
        map_x = self.map_position[0]
        map_y = self.map_position[1]

        if self.type == TileType.Straight:
            exit_quadrant = self.orientation_quadrant
        elif self.type == TileType.LeftTurn:
            exit_quadrant = (self.orientation_quadrant + 1) % 4
        elif self.type == TileType.RightTurn:
            exit_quadrant = (self.orientation_quadrant - 1) % 4

        offset_x, offset_y = self._DIRECTION_OFFSETS[exit_quadrant]
        return (map_x + offset_x, map_y + offset_y)


    @staticmethod
    def rotate(rotate_rad,x,y):

        rotate_center = (0.3,0.3)

        x_rotated = [(x[i] - rotate_center[0]) * np.cos(rotate_rad) - (y[i] - rotate_center[1]) * np.sin(rotate_rad) + rotate_center[0] for i in range(len(x))]

        y_rotated = [(x[i] - rotate_center[0]) * np.sin(rotate_rad) + (y[i] - rotate_center[1]) * np.cos(rotate_rad) + rotate_center[1] for i in range(len(x))]

        return x_rotated, y_rotated

    
    def draw_points(points,current_point=None):

        # work entirely in pixel space (world units * 100) and snap the bounds outward to the nearest 60px tile boundary
        xs = [point[0] * 100 for point in points]
        ys = [point[1] * 100 for point in points]
        min_x = math.floor(min(xs) / 60) * 60
        max_x = math.ceil(max(xs) / 60) * 60
        min_y = math.floor(min(ys) / 60) * 60
        max_y = math.ceil(max(ys) / 60) * 60

        #print(f"Drawing LaneSegment with min_x: {min_x}, max_x: {max_x}, min_y: {min_y}, max_y: {max_y}")

        # Create a blank image with some padding
        pad = 10
        image_width = (max_x - min_x) + 2 * pad
        image_height = (max_y - min_y) + 2 * pad
        image = np.zeros((image_height, image_width, 3), dtype=np.uint8)

        points = np.array(points)
        points_image = points * np.array([100, -100, 1]) - np.array([min_x, -max_y, 0]) + np.array([pad, pad, 0])
        image[points_image[:, 1].astype(int), points_image[:, 0].astype(int)] = (0, 255, 0)

        #for point in points:
        #    px = int(point[0] * 100 - min_x + pad)
        #    py = int(max_y - (point[1] * 100) + pad)
        #    cv2.circle(image, (px, py), 2, (0, 255, 0), -1)

        # draw tile borders every 60px
        for i in range(min_x, max_x, 60):
            for j in range(min_y, max_y, 60):
                top_left = (int(i - min_x + pad), int(max_y -j) + pad)
                bottom_right = (int(i - min_x + pad + 60), int(max_y - j - 60) + pad)
                cv2.rectangle(image, top_left, bottom_right, (255, 255, 255), 1)

        if current_point is not None:
            px = int(current_point[0] * 100 - min_x + pad)
            py = int(max_y - (current_point[1] * 100) + pad)
            p_theta = current_point[2]

            #draw a triangle indicating the orientation of the point
            rect_length = 10 
            triangle_cnt = np.array([
                (px + int(rect_length * np.cos(p_theta)), py - int(rect_length * np.sin(p_theta))),
                (px + int(rect_length * np.cos(p_theta + 2.5)), py - int(rect_length * np.sin(p_theta + 2.5))),
                (px + int(rect_length * np.cos(p_theta - 2.5)), py - int(rect_length * np.sin(p_theta - 2.5)))
            ])
            cv2.drawContours(image, [triangle_cnt], 0, (0, 0, 255), -1)

        return image


