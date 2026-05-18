"""
LiDARSimulator: Simulates LiDAR point cloud for SLAM and navigation
"""
import math
import random
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class PointCloud:
    """LiDAR point cloud data"""
    points: List[tuple]  # List of (distance, angle) tuples


@dataclass
class Obstacle:
    """Detected obstacle"""
    x: float
    y: float
    confidence: float


class LiDARSimulator:
    """
    Simulates LiDAR scanner that detects field boundaries and crop rows
    For simplicity: simulates 16 angular sectors with distance measurements
    """
    
    def __init__(self, field_width: float = 500.0, field_height: float = 300.0,
                 field_offset_x: float = 50.0, field_offset_y: float = 50.0):
        """
        Args:
            field_width: Field width in meters
            field_height: Field height in meters
            field_offset_x, field_offset_y: Field origin offset
        """
        self.field_width = field_width
        self.field_height = field_height
        self.field_offset_x = field_offset_x
        self.field_offset_y = field_offset_y
        
        # LiDAR specifications
        self.num_channels = 16
        self.max_range = 100.0  # meters
        self.measurement_noise = 0.02  # meters
        
        # Crop row spacing (6 meters)
        self.row_spacing = 6.0
    
    def get_scan(self, true_x: float, true_y: float, true_heading: float) -> PointCloud:
        """
        Get LiDAR scan data
        Args:
            true_x, true_y: True position in local coordinates
            true_heading: True heading in radians
        Returns:
            PointCloud with 16 distance measurements
        """
        points = []
        
        for i in range(self.num_channels):
            # Angle for this channel (0 to 2π, evenly spaced)
            angle = (i / self.num_channels) * 2 * math.pi + true_heading
            
            # Calculate distance to nearest boundary or obstacle
            distance = self._calculate_distance_to_obstacle(true_x, true_y, angle)
            
            # Add realistic measurement noise
            distance_noisy = distance + random.gauss(0, self.measurement_noise)
            distance_noisy = max(0, min(self.max_range, distance_noisy))
            
            points.append((distance_noisy, angle))
        
        return PointCloud(points=points)
    
    def _calculate_distance_to_obstacle(self, x: float, y: float, angle: float) -> float:
        """
        Calculate distance from position to nearest obstacle in given direction
        Simplified: just checks field boundaries and row proximity
        """
        # Direction unit vector
        dx = math.cos(angle)
        dy = math.sin(angle)
        
        min_distance = self.max_range
        
        # Check intersection with field boundaries
        # Left boundary (X = field_offset_x)
        if dx > 0.001:
            t = (self.field_offset_x - x) / dx
            if t > 0 and t < min_distance:
                min_distance = t
        elif dx < -0.001:
            t = (self.field_offset_x + self.field_width - x) / dx
            if t > 0 and t < min_distance:
                min_distance = t
        
        # Check bottom and top boundaries
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
        """
        Detect lateral offset from nearest crop row
        Returns offset in meters (positive = right of center, negative = left)
        """
        # Simplified: look for nearest LiDAR returns at angles 90° to 270° (left/right)
        # These indicate proximity to crop rows
        
        # Look at forward sectors and estimate row offset
        left_distances = []
        right_distances = []
        
        for distance, angle in scan_points.points:
            # Normalize angle to 0-2π
            norm_angle = angle % (2 * math.pi)
            
            # Left sector (π/2 ± π/4)
            if math.pi / 4 < norm_angle < 3 * math.pi / 4:
                left_distances.append(distance)
            
            # Right sector (3π/2 ± π/4)
            if 5 * math.pi / 4 < norm_angle < 7 * math.pi / 4:
                right_distances.append(distance)
        
        if not left_distances or not right_distances:
            return None
        
        # Calculate average distances
        avg_left = sum(left_distances) / len(left_distances)
        avg_right = sum(right_distances) / len(right_distances)
        
        # Offset as difference
        offset = (avg_left - avg_right) / 2.0
        
        # Only return if offset is reasonable (within 3 meters of expected row spacing)
        if abs(offset) < 3.0:
            return offset
        
        return None
    
    def detect_obstacles(self, scan_points: PointCloud, max_distance: float = 5.0) -> List[Obstacle]:
        """
        Detect obstacles in front of vehicle
        Args:
            scan_points: LiDAR scan
            max_distance: Maximum distance to consider as obstacle
        Returns:
            List of detected obstacles
        """
        obstacles = []
        
        for distance, angle in scan_points.points:
            if distance < max_distance:
                # Convert to local coordinates
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                obstacles.append(Obstacle(x=x, y=y, confidence=1.0))
        
        return obstacles
    
    def try_get_position_correction(self, scan_points: PointCloud) -> Optional[tuple]:
        """
        Try to detect known landmarks and return position correction
        Returns (dx, dy) tuple or None
        """
        # Simplified: just return row offset as Y correction
        offset = self.detect_row_offset(scan_points)
        if offset is not None:
            return (0.0, offset)
        return None
