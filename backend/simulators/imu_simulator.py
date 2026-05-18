"""IMU Simulator"""
import numpy as np
from typing import Dict, Any


class IMUSimulator:
    """Simulates IMU sensor data"""

    def __init__(self):
        """Initialize IMU simulator"""
        self.cycle = 0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = -9.81  # Gravity
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.temperature = 25.0

    def read(self) -> Dict[str, Any]:
        """Read IMU data"""
        self.cycle += 1
        
        # Simulate circular motion
        accel_x = 0.5 * np.sin(self.cycle * 0.02)
        accel_y = 0.3 * np.cos(self.cycle * 0.02)
        
        # Simulate rotation
        gyro_z = 0.1 * np.sin(self.cycle * 0.01)
        
        # Add noise
        accel_x += np.random.normal(0, 0.01)
        accel_y += np.random.normal(0, 0.01)
        accel_z = -9.81 + np.random.normal(0, 0.01)
        gyro_z += np.random.normal(0, 0.001)
        
        return {
            'accel_x': accel_x,
            'accel_y': accel_y,
            'accel_z': accel_z,
            'gyro_x': np.random.normal(0, 0.001),
            'gyro_y': np.random.normal(0, 0.001),
            'gyro_z': gyro_z,
            'temperature': self.temperature + np.random.normal(0, 0.1)
        }
