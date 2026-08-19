import json
import math
import os
import threading
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import rospy
from duckietown_msgs.msg import WheelEncoderStamped

class Tracker:
    """Track Duckiebot pose from wheel encoders and build a coarse tile map."""

    def __init__(
        self,
        vehicle_name: str,
        wheel_radius: float = 0.0318,
        ticks_per_revolution: int = 135,
        baseline: float = 0.1,
        segment_distance: float = 0.60,
        significant_turn_threshold_rad: float = math.radians(90.0),
        cell_size: float = 0.60,
        map_output_path: Optional[str] = None,
    ):
        self.wheel_radius = float(wheel_radius)
        self.ticks_per_revolution = int(ticks_per_revolution)
        self.baseline = float(baseline)
        self.segment_distance = float(segment_distance)
        self.significant_turn_threshold_rad = float(significant_turn_threshold_rad)
        self.cell_size = float(cell_size)

        self._left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"
        self.vehicle_name = vehicle_name

        default_map_file = f"tracker_map_{vehicle_name}.json"
        self.map_output_path = map_output_path or os.path.join(os.getcwd(), default_map_file)

        self._lock = threading.Lock()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self._left_tick: Optional[int] = None
        self._right_tick: Optional[int] = None
        self._prev_left_tick: Optional[int] = None
        self._prev_right_tick: Optional[int] = None

        self._segment_distance_acc = 0.0
        self._segment_turn_acc = 0.0
        self._segment_left_distance_acc = 0.0
        self._segment_right_distance_acc = 0.0
        self._segment_start_heading = 0.0
        self._segment_start_pos: Tuple[float, float] = (0.0, 0.0)
        self._last_step_is_turning = False

        self._curvature_turn_threshold = 1.0
        self._min_step_turn_rad = math.radians(0.8)
        self._turn_commit_min_rad = math.radians(70.0)
        self._straight_commit_min_distance = 0.45

        self._map_cells: Dict[Tuple[int, int], Dict[str, object]] = {}

        self.sub_left = rospy.Subscriber(
            self._left_encoder_topic,
            WheelEncoderStamped,
            self.callback_left,
            queue_size=1,
        )
        self.sub_right = rospy.Subscriber(
            self._right_encoder_topic,
            WheelEncoderStamped,
            self.callback_right,
            queue_size=1,
        )

        rospy.on_shutdown(self._on_shutdown)

    def callback_left(self, msg: WheelEncoderStamped):
        with self._lock:
            self._left_tick = int(msg.data)
            self._try_integrate_odometry()

    def callback_right(self, msg: WheelEncoderStamped):
        with self._lock:
            self._right_tick = int(msg.data)
            self._try_integrate_odometry()

    def _try_integrate_odometry(self):
        if self._left_tick is None or self._right_tick is None:
            return

        if self._prev_left_tick is None or self._prev_right_tick is None:
            self._prev_left_tick = self._left_tick
            self._prev_right_tick = self._right_tick
            self._segment_start_heading = self.theta
            self._segment_start_pos = (self.x, self.y)
            return

        delta_left_ticks = self._left_tick - self._prev_left_tick
        delta_right_ticks = self._right_tick - self._prev_right_tick

        if delta_left_ticks == 0 and delta_right_ticks == 0:
            return

        self._prev_left_tick = self._left_tick
        self._prev_right_tick = self._right_tick

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

        self._segment_distance_acc += abs(d_center)
        self._segment_turn_acc += abs(delta_theta)
        self._segment_left_distance_acc += d_left
        self._segment_right_distance_acc += d_right
        self._last_step_is_turning = self._is_turning_step(d_center, delta_theta)

        commit_type = self._segment_should_commit()
        if commit_type is not None:
            self._commit_segment_to_map(commit_type)

    def _segment_should_commit(self) -> Optional[str]:
        if self._segment_distance_acc <= 0.0 and self._segment_turn_acc <= 0.0:
            return None

        if self._segment_turn_acc >= self.significant_turn_threshold_rad:
            return "curve"

        if self._segment_distance_acc >= self.segment_distance:
            if self._segment_is_curve_like():
                return "curve"
            return "straight"

        # Transition: robot starts turning before reaching full straight length.
        if self._last_step_is_turning and self._segment_distance_acc >= self._straight_commit_min_distance:
            if not self._segment_is_curve_like():
                return "straight"

        # Transition: robot exits a turn before reaching exact 90 degrees.
        if (not self._last_step_is_turning) and self._segment_turn_acc >= self._turn_commit_min_rad:
            if self._segment_is_curve_like():
                return "curve"

        return None

    def _commit_segment_to_map(self, forced_type: Optional[str] = None):
        if self._segment_distance_acc <= 0.0 and self._segment_turn_acc <= 0.0:
            return

        segment_is_curve = forced_type == "curve"
        if forced_type is None:
            segment_is_curve = self._segment_is_curve_like()

        start_x, start_y = self._segment_start_pos
        start_theta = self._segment_start_heading

        tile_type = "curve" if segment_is_curve else "straight"
        rotation = self._heading_to_rotation(start_theta)

        curve_kind = None
        turn_direction = None
        if segment_is_curve:
            turn_direction = "left" if self._segment_right_distance_acc > self._segment_left_distance_acc else "right"
            curve_kind = self._classify_curve_kind(
                self._segment_left_distance_acc,
                self._segment_right_distance_acc,
            )
            self.x, self.y, self.theta = self._snap_curve_pose(start_x, start_y, start_theta, turn_direction)
        else:
            self.x, self.y, self.theta = self._snap_straight_pose(start_x, start_y, start_theta)

        gx, gy = self._pose_to_grid(self.x, self.y)
        cell_key = (gx, gy)
        self._map_cells[cell_key] = {
            "type": tile_type,
            "rotation": rotation,
            "curve_kind": curve_kind,
            "turn_direction": turn_direction,
            "visits": int(self._map_cells.get(cell_key, {}).get("visits", 0)) + 1,
        }

        self._segment_distance_acc = 0.0
        self._segment_turn_acc = 0.0
        self._segment_left_distance_acc = 0.0
        self._segment_right_distance_acc = 0.0
        self._segment_start_heading = self.theta
        self._segment_start_pos = (self.x, self.y)
        self._last_step_is_turning = False

    def _segment_is_curve_like(self) -> bool:
        if self._segment_turn_acc >= self._turn_commit_min_rad:
            return True
        if self._segment_distance_acc <= 1e-6:
            return self._segment_turn_acc > 0.0
        curvature = self._segment_turn_acc / self._segment_distance_acc
        return curvature >= self._curvature_turn_threshold

    def _is_turning_step(self, d_center: float, delta_theta: float) -> bool:
        abs_turn = abs(delta_theta)
        if abs_turn < self._min_step_turn_rad:
            return False
        curvature = abs_turn / (abs(d_center) + 1e-6)
        return curvature >= self._curvature_turn_threshold

    def _snap_straight_pose(self, start_x: float, start_y: float, start_theta: float) -> Tuple[float, float, float]:
        x = start_x + self.segment_distance * math.cos(start_theta)
        y = start_y + self.segment_distance * math.sin(start_theta)
        theta = self._wrap_angle(start_theta)
        return x, y, theta

    def _snap_curve_pose(
        self,
        start_x: float,
        start_y: float,
        start_theta: float,
        turn_direction: str,
    ) -> Tuple[float, float, float]:
        signed_turn = self.significant_turn_threshold_rad if turn_direction == "left" else -self.significant_turn_threshold_rad
        measured_turn = max(self._segment_turn_acc, 1e-6)
        r_est = self._segment_distance_acc / measured_turn

        x = start_x + r_est * (math.sin(start_theta + signed_turn) - math.sin(start_theta))
        y = start_y - r_est * (math.cos(start_theta + signed_turn) - math.cos(start_theta))
        theta = self._wrap_angle(start_theta + signed_turn)
        return x, y, theta

    def _pose_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int(round(x / self.cell_size))
        gy = int(round(y / self.cell_size))
        return gx, gy

    def _heading_to_rotation(self, heading_rad: float) -> int:
        heading = self._wrap_angle(heading_rad)
        if -math.pi / 4 <= heading < math.pi / 4:
            return 1
        if math.pi / 4 <= heading < 3 * math.pi / 4:
            return 2
        if heading >= 3 * math.pi / 4 or heading < -3 * math.pi / 4:
            return 3
        return 4

    def _classify_curve_kind(self, left_distance: float, right_distance: float) -> str:
        abs_left = abs(left_distance)
        abs_right = abs(right_distance)
        if abs_left <= 1e-6 and abs_right <= 1e-6:
            return "outer"

        max_dist = max(abs_left, abs_right)
        min_dist = min(abs_left, abs_right)
        ratio = min_dist / max_dist if max_dist > 0.0 else 1.0

        # Lower ratio means one wheel moved much less than the other wheel.
        return "inner" if ratio < 0.7 else "outer"

    def _build_dense_map(self) -> Dict[str, object]:
        if not self._map_cells:
            return {
                "origin": {"x": 0, "y": 0},
                "cell_size": self.cell_size,
                "grid": [],
            }

        all_x = [k[0] for k in self._map_cells.keys()]
        all_y = [k[1] for k in self._map_cells.keys()]
        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)

        width = max_x - min_x + 1
        height = max_y - min_y + 1

        grid = [[None for _ in range(width)] for _ in range(height)]
        for (gx, gy), cell in self._map_cells.items():
            row = max_y - gy
            col = gx - min_x
            grid[row][col] = cell

        return {
            "origin": {"x": min_x, "y": min_y},
            "cell_size": self.cell_size,
            "grid": grid,
        }

    def get_pose(self) -> Tuple[float, float, float]:
        with self._lock:
            return self.x, self.y, self.theta

    def save_map(self, path: Optional[str] = None):
        output_path = path or self.map_output_path
        with self._lock:
            if self._segment_distance_acc > 0.0 or self._segment_turn_acc > 0.0:
                self._commit_segment_to_map(forced_type="curve" if self._segment_is_curve_like() else "straight")

            payload = {
                "pose": {
                    "x": self.x,
                    "y": self.y,
                    "theta": self.theta,
                },
                "parameters": {
                    "wheel_radius": self.wheel_radius,
                    "ticks_per_revolution": self.ticks_per_revolution,
                    "baseline": self.baseline,
                    "segment_distance": self.segment_distance,
                    "significant_turn_threshold_rad": self.significant_turn_threshold_rad,
                },
                "map": self._build_dense_map(),
            }

        os.makedirs(os.path.dirname(output_path), exist_ok=True) if os.path.dirname(output_path) else None
        with open(output_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)

        rospy.loginfo(f"Tracker map saved to {output_path}")

    def plot_map(
        self,
        output_path: Optional[str] = None,
        cell_px: int = 80,
        line_thickness: int = 4,
        robot_color: Optional[Tuple[int, int, int]] = None,
    ) -> np.ndarray:
        """Render the tracked grid map as an image and optionally save it."""
        with self._lock:
            dense_map = self._build_dense_map()
            pose_x, pose_y, pose_theta = self.x, self.y, self.theta

        grid = dense_map["grid"]
        if not grid:
            empty = np.zeros((cell_px, cell_px, 3), dtype=np.uint8)
            if output_path:
                self._ensure_parent_dir(output_path)
                cv2.imwrite(output_path, empty)
            return empty

        rows = len(grid)
        cols = len(grid[0]) if rows > 0 else 0
        img_h = max(1, rows * cell_px)
        img_w = max(1, cols * cell_px)
        img = np.zeros((img_h, img_w, 3), dtype=np.uint8)

        box_color = (70, 70, 70)
        track_color = (255, 255, 255)

        for r, row in enumerate(grid):
            for c, cell in enumerate(row):
                x0 = c * cell_px
                y0 = r * cell_px
                x1 = x0 + cell_px
                y1 = y0 + cell_px

                cv2.rectangle(img, (x0, y0), (x1, y1), box_color, 1)
                if cell is None:
                    continue

                cell_type = cell.get("type", "straight")
                rotation = int(cell.get("rotation", 1))

                if cell_type == "straight":
                    self._draw_straight_cell(img, x0, y0, x1, y1, rotation, track_color, line_thickness)
                else:
                    turn_direction = str(cell.get("turn_direction", "right"))
                    self._draw_curve_cell(
                        img,
                        x0,
                        y0,
                        x1,
                        y1,
                        rotation,
                        turn_direction,
                        track_color,
                        line_thickness,
                    )

        self._draw_robot_triangle(
            img,
            dense_map,
            pose_x,
            pose_y,
            pose_theta,
            cell_px,
            line_thickness,
            robot_color,
        )

        if output_path is None:
            output_path = os.path.join(os.getcwd(), f"tracker_map_{self.vehicle_name}.png")

        if output_path:
            self._ensure_parent_dir(output_path)
            cv2.imwrite(output_path, img)
            rospy.loginfo(f"Tracker map plot saved to {output_path}")

        return img

    def _draw_straight_cell(
        self,
        img: np.ndarray,
        x0: int,
        y0: int,
        x1: int,
        y1: int,
        rotation: int,
        color: Tuple[int, int, int],
        thickness: int,
    ):
        cx = (x0 + x1) // 2
        cy = (y0 + y1) // 2
        if rotation in (1, 3):
            cv2.line(img, (x0, cy), (x1, cy), color, thickness)
        else:
            cv2.line(img, (cx, y0), (cx, y1), color, thickness)

    def _draw_curve_cell(
        self,
        img: np.ndarray,
        x0: int,
        y0: int,
        x1: int,
        y1: int,
        rotation: int,
        turn_direction: str,
        color: Tuple[int, int, int],
        thickness: int,
    ):
        heading = self._rotation_to_heading(rotation)
        entry = self._opposite_direction(heading)
        turn = "left" if turn_direction == "left" else "right"
        exit_dir = self._turn_direction(heading, turn)

        cx = (x0 + x1) / 2.0
        cy = (y0 + y1) / 2.0
        half = (x1 - x0) / 2.0

        edge_mid = {
            "N": (cx, float(y0)),
            "E": (float(x1), cy),
            "S": (cx, float(y1)),
            "W": (float(x0), cy),
        }

        corner = self._corner_from_edges(entry, exit_dir)
        corners = {
            "NW": (float(x0), float(y0)),
            "NE": (float(x1), float(y0)),
            "SE": (float(x1), float(y1)),
            "SW": (float(x0), float(y1)),
        }

        center = corners[corner]
        p_start = edge_mid[entry]
        p_end = edge_mid[exit_dir]

        a0 = math.atan2(p_start[1] - center[1], p_start[0] - center[0])
        a1 = math.atan2(p_end[1] - center[1], p_end[0] - center[0])

        delta_candidates = [math.pi / 2.0, -math.pi / 2.0]
        best_delta = delta_candidates[0]
        best_error = float("inf")
        for delta in delta_candidates:
            x_end = center[0] + half * math.cos(a0 + delta)
            y_end = center[1] + half * math.sin(a0 + delta)
            err = (x_end - p_end[0]) ** 2 + (y_end - p_end[1]) ** 2
            if err < best_error:
                best_error = err
                best_delta = delta

        points = []
        steps = 20
        for i in range(steps + 1):
            t = i / float(steps)
            a = a0 + best_delta * t
            px = int(round(center[0] + half * math.cos(a)))
            py = int(round(center[1] + half * math.sin(a)))
            points.append([px, py])

        cv2.polylines(img, [np.array(points, dtype=np.int32)], False, color, thickness)

    def _draw_robot_triangle(
        self,
        img: np.ndarray,
        dense_map: Dict[str, object],
        pose_x: float,
        pose_y: float,
        pose_theta: float,
        cell_px: int,
        line_thickness: int,
        robot_color: Optional[Tuple[int, int, int]],
    ):
        grid = dense_map.get("grid", [])
        if not grid:
            return

        origin = dense_map.get("origin", {"x": 0, "y": 0})
        origin_x = int(origin.get("x", 0))
        origin_y = int(origin.get("y", 0))
        rows = len(grid)
        cols = len(grid[0]) if rows else 0
        if rows == 0 or cols == 0:
            return

        gx, gy = self._pose_to_grid(pose_x, pose_y)
        col = gx - origin_x
        row = (rows - 1) - (gy - origin_y)

        if row < 0 or row >= rows or col < 0 or col >= cols:
            return

        cell = grid[row][col]
        if cell is None:
            return

        x0 = col * cell_px
        y0 = row * cell_px
        x1 = x0 + cell_px
        y1 = y0 + cell_px

        marker_center = self._project_pose_to_cell_line(cell, x0, y0, x1, y1)
        self._draw_oriented_triangle(
            img=img,
            center=marker_center,
            heading_rad=pose_theta,
            size=max(8, int(cell_px * 0.22)),
            color=robot_color if robot_color is not None else (0, 255, 0),
            thickness=max(1, line_thickness // 2),
        )

    def _project_pose_to_cell_line(
        self,
        cell: Dict[str, object],
        x0: int,
        y0: int,
        x1: int,
        y1: int,
    ) -> Tuple[int, int]:
        cell_type = str(cell.get("type", "straight"))
        rotation = int(cell.get("rotation", 1))
        cx = int(round((x0 + x1) * 0.5))
        cy = int(round((y0 + y1) * 0.5))

        if cell_type == "straight":
            if rotation in (1, 3):
                return cx, cy
            return cx, cy

        turn_direction = str(cell.get("turn_direction", "right"))
        heading = self._rotation_to_heading(rotation)
        entry = self._opposite_direction(heading)
        turn = "left" if turn_direction == "left" else "right"
        exit_dir = self._turn_direction(heading, turn)
        corner = self._corner_from_edges(entry, exit_dir)

        half = (x1 - x0) * 0.5
        corners = {
            "NW": (float(x0), float(y0)),
            "NE": (float(x1), float(y0)),
            "SE": (float(x1), float(y1)),
            "SW": (float(x0), float(y1)),
        }
        center = corners[corner]
        target = {
            "N": (float(cx), float(y0)),
            "E": (float(x1), float(cy)),
            "S": (float(cx), float(y1)),
            "W": (float(x0), float(cy)),
        }[exit_dir]

        a0 = math.atan2(target[1] - center[1], target[0] - center[0])
        px = int(round(center[0] + half * math.cos(a0)))
        py = int(round(center[1] + half * math.sin(a0)))
        return px, py

    @staticmethod
    def _draw_oriented_triangle(
        img: np.ndarray,
        center: Tuple[int, int],
        heading_rad: float,
        size: int,
        color: Tuple[int, int, int],
        thickness: int,
    ):
        cx, cy = center
        heading_screen = -heading_rad

        tip = (
            int(round(cx + size * math.cos(heading_screen))),
            int(round(cy + size * math.sin(heading_screen))),
        )

        rear_dist = size * 0.75
        half_width = size * 0.55
        back_cx = cx - rear_dist * math.cos(heading_screen)
        back_cy = cy - rear_dist * math.sin(heading_screen)

        left = (
            int(round(back_cx + half_width * math.cos(heading_screen + math.pi / 2.0))),
            int(round(back_cy + half_width * math.sin(heading_screen + math.pi / 2.0))),
        )
        right = (
            int(round(back_cx + half_width * math.cos(heading_screen - math.pi / 2.0))),
            int(round(back_cy + half_width * math.sin(heading_screen - math.pi / 2.0))),
        )

        pts = np.array([tip, left, right], dtype=np.int32)
        cv2.fillConvexPoly(img, pts, color)
        cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 0), thickness=max(1, thickness))

    @staticmethod
    def _rotation_to_heading(rotation: int) -> str:
        if rotation == 1:
            return "E"
        if rotation == 2:
            return "N"
        if rotation == 3:
            return "W"
        return "S"

    @staticmethod
    def _opposite_direction(direction: str) -> str:
        return {"N": "S", "S": "N", "E": "W", "W": "E"}[direction]

    @staticmethod
    def _turn_direction(direction: str, turn: str) -> str:
        right_map = {"N": "E", "E": "S", "S": "W", "W": "N"}
        left_map = {"N": "W", "W": "S", "S": "E", "E": "N"}
        return left_map[direction] if turn == "left" else right_map[direction]

    @staticmethod
    def _corner_from_edges(edge_a: str, edge_b: str) -> str:
        edge_set = {edge_a, edge_b}
        if edge_set == {"N", "W"}:
            return "NW"
        if edge_set == {"N", "E"}:
            return "NE"
        if edge_set == {"S", "E"}:
            return "SE"
        return "SW"

    @staticmethod
    def _ensure_parent_dir(path: str):
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)

    def _on_shutdown(self):
        try:
            self.save_map()
            self.plot_map(output_path="my_map.png")
        except Exception as exc:
            rospy.logwarn(f"Failed to save tracker map on shutdown: {exc}")

    @staticmethod
    def _wrap_angle(angle_rad: float) -> float:
        return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi
