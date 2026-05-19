import math
import random
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class PointCloud:
    points: List[tuple]


@dataclass
class Obstacle:
    x: float
    y: float
    confidence: float


class LiDARSimulator:
    def __init__(self, field_width: float = 500.0, field_height: float = 300.0,
                 field_offset_x: float = 50.0, field_offset_y: float = 50.0):
        self.field_width = field_width
        self.field_height = field_height
        self.field_offset_x = field_offset_x
        self.field_offset_y = field_offset_y

        self.num_channels = 16
        self.max_range = 100.0
        self.measurement_noise = 0.02

        self.row_spacing = 6.0
    
    def get_scan(self, true_x: float, true_y: float, true_heading: float) -> PointCloud:
        points = []

        for i in range(self.num_channels):
            angle = (i / self.num_channels) * 2 * math.pi + true_heading

            distance = self._calculate_distance_to_obstacle(true_x, true_y, angle)

            distance_noisy = distance + random.gauss(0, self.measurement_noise)
            distance_noisy = max(0, min(self.max_range, distance_noisy))

            points.append((distance_noisy, angle))

        return PointCloud(points=points)
    
    def _calculate_distance_to_obstacle(self, x: float, y: float, angle: float) -> float:
        dx = math.cos(angle)
        dy = math.sin(angle)

        min_distance = self.max_range

        if dx > 0.001:
            t = (self.field_offset_x - x) / dx
            if t > 0 and t < min_distance:
                min_distance = t
        elif dx < -0.001:
            t = (self.field_offset_x + self.field_width - x) / dx
            if t > 0 and t < min_distance:
                min_distance = t

        if dy > 0.001:
            t = (self.field_offset_y + self.field_height - y) / dy
            if t > 0 and t < min_distance:
                min_distance = t
        elif dy < -0.001:
            t = (self.field_offset_y - y) / dy
            if t > 0 and t < min_distance:
                min_distance = t

        return min_distance
    
    def detect_row_offset(self, scan_points: PointCloud) -> Optional[float]:
        left_distances = []
        right_distances = []

        for distance, angle in scan_points.points:
            norm_angle = angle % (2 * math.pi)

            if math.pi / 4 < norm_angle < 3 * math.pi / 4:
                left_distances.append(distance)

            if 5 * math.pi / 4 < norm_angle < 7 * math.pi / 4:
                right_distances.append(distance)

        if not left_distances or not right_distances:
            return None

        avg_left = sum(left_distances) / len(left_distances)
        avg_right = sum(right_distances) / len(right_distances)

        offset = (avg_left - avg_right) / 2.0

        if abs(offset) < 3.0:
            return offset

        return None
    
    def detect_obstacles(self, scan_points: PointCloud, max_distance: float = 5.0) -> List[Obstacle]:
        obstacles = []

        for distance, angle in scan_points.points:
            if distance < max_distance:
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                obstacles.append(Obstacle(x=x, y=y, confidence=1.0))

        return obstacles
    
    def try_get_position_correction(self, scan_points: PointCloud) -> Optional[tuple]:
        offset = self.detect_row_offset(scan_points)
        if offset is not None:
            return (0.0, offset)
        return None
