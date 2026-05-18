"""LiDAR Simulator"""
import numpy as np
from typing import Dict, Any, Tuple, Optional


class LiDARSimulator:
    """Simulates LiDAR scanner and SLAM"""

    def __init__(self):
        """Initialize LiDAR simulator"""
        self.cycle = 0
        self.num_points = 16
        self.position_correction_x = 0.0
        self.position_correction_y = 0.0

    def read(self) -> Optional[Tuple[float, float]]:
        """
        Read LiDAR data and return position correction
        
        Returns:
            Tuple of (dx, dy) correction or None
        """
        self.cycle += 1
        
        # LiDAR SLAM provides position corrections
        # Simulate SLAM correction that accumulates over time
        dx = self.cycle * 0.001 + np.random.normal(0, 0.01)
        dy = self.cycle * 0.0005 + np.random.normal(0, 0.01)
        
        return (dx, dy)

    def get_point_cloud(self) -> np.ndarray:
        """
        Get simulated point cloud
        
        Returns:
            Array of (x, y, z) points
        """
        # Simulate 16 channels of LiDAR data
        angles = np.linspace(-15, 15, self.num_points)
        distances = np.random.uniform(5, 100, self.num_points)
        
        points = []
        for angle, distance in zip(angles, distances):
            rad = np.radians(angle)
            x = distance * np.cos(rad)
            z = distance * np.sin(rad)
            y = np.random.uniform(-2, 2)
            points.append([x, y, z])
        
        return np.array(points)

    def filter_ground_points(self, points: np.ndarray) -> np.ndarray:
        """Filter ground points from cloud"""
        # Remove points with z < 0.5 (ground level)
        return points[points[:, 2] > 0.5]

    def detect_obstacles(self, points: np.ndarray) -> list:
        """Detect obstacles in point cloud"""
        obstacles = []
        filtered = self.filter_ground_points(points)
        
        for point in filtered:
            distance = np.sqrt(point[0]**2 + point[1]**2)
            if distance < 50:  # Within 50m
                obstacles.append({
                    'x': point[0],
                    'y': point[1],
                    'z': point[2],
                    'distance': distance
                })
        
        return obstacles
