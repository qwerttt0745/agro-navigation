"""Vehicle Simulator"""
import numpy as np
from typing import Dict, Any


class VehicleSimulator:
    """Simulates vehicle dynamics"""

    def __init__(self):
        """Initialize vehicle simulator"""
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.speed = 0.0
        self.steering_angle = 0.0
        self.wheelbase = 2.5  # meters
        self.max_speed = 15.0  # m/s

    def step(self, steering_angle_deg: float, speed_mps: float, dt: float = 0.1):
        """
        Simulate one timestep of vehicle dynamics
        
        Args:
            steering_angle_deg: Steering angle in degrees
            speed_mps: Target speed in m/s
            dt: Timestep in seconds
        """
        # Limit steering angle
        self.steering_angle = np.clip(steering_angle_deg, -35, 35)
        
        # Update speed (first-order lag)
        self.speed += (speed_mps - self.speed) * 0.1
        self.speed = np.clip(self.speed, 0, self.max_speed)
        
        if abs(self.speed) < 0.01:
            return
        
        # Bicycle model
        steering_rad = np.radians(self.steering_angle)
        
        # Update heading (Ackermann steering)
        delta_heading = (self.speed / self.wheelbase) * np.tan(steering_rad) * dt
        self.heading += delta_heading
        
        # Update position
        cos_h = np.cos(self.heading)
        sin_h = np.sin(self.heading)
        
        self.x += self.speed * sin_h * dt
        self.y += self.speed * cos_h * dt

    def get_state(self) -> Dict[str, Any]:
        """Get current vehicle state"""
        return {
            'x': self.x,
            'y': self.y,
            'heading': self.heading,
            'speed': self.speed,
            'steering_angle': self.steering_angle
        }
