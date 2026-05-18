"""Dead Reckoning module for autonomous positioning"""
import numpy as np
from typing import Tuple


class DeadReckoningModule:
    """Dead Reckoning using IMU integration"""

    def __init__(self):
        """Initialize Dead Reckoning module"""
        self.x = 0.0  # Position X
        self.y = 0.0  # Position Y
        self.heading = 0.0  # Current heading (radians)
        self.distance_traveled = 0.0  # Total distance
        self.accumulated_error = 0.0  # Accumulated positioning error
        self.gyro_bias = 0.001  # Gyro drift bias (rad/s)
        self.active = False
        self.start_distance = 0.0

    def activate(self):
        """Activate Dead Reckoning"""
        self.active = True
        self.start_distance = self.distance_traveled
        self.accumulated_error = 0.0

    def deactivate(self):
        """Deactivate Dead Reckoning"""
        self.active = False

    def integrate_imu(self, accel_x: float, accel_y: float, 
                     accel_z: float, gyro_x: float, gyro_y: float, 
                     gyro_z: float, dt: float = 0.1) -> Tuple[float, float, float]:
        """
        Integrate IMU data for Dead Reckoning
        
        Returns:
            (x, y, heading) - Updated position and heading
        """
        if not self.active:
            return self.x, self.y, self.heading
        
        # Rotate accelerations to navigation frame
        cos_h = np.cos(self.heading)
        sin_h = np.sin(self.heading)
        
        ax_nav = accel_x * cos_h - accel_y * sin_h
        ay_nav = accel_x * sin_h + accel_y * cos_h
        
        # Calculate distance traveled this cycle
        distance = np.sqrt(ax_nav**2 + ay_nav**2) * (dt**2) / 2
        self.distance_traveled += distance
        
        # Update position (heading = 0 means North, positive Y)
        self.y += distance * np.cos(self.heading)
        self.x += distance * np.sin(self.heading)
        
        # Update heading with gyro
        self.heading += gyro_z * dt
        
        # Accumulate error: error ∝ gyro_bias × distance_traveled
        cycle_distance = self.distance_traveled - self.start_distance
        self.accumulated_error = self.gyro_bias * cycle_distance
        
        return self.x, self.y, self.heading

    def get_accumulated_error(self) -> float:
        """Get accumulated positioning error"""
        return self.accumulated_error

    def is_error_within_threshold(self, threshold: float = 0.3) -> bool:
        """Check if error is within threshold (30cm per 100m)"""
        return self.accumulated_error <= threshold

    def reset(self):
        """Reset Dead Reckoning"""
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.distance_traveled = 0.0
        self.accumulated_error = 0.0
        self.active = False
