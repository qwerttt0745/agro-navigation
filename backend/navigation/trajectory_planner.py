"""Trajectory planning and steering command generation"""
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class Waypoint:
    """Single waypoint"""
    x: float
    y: float
    heading: float = 0.0


@dataclass
class SteeringCommand:
    """Steering command"""
    angle_deg: float  # Steering angle (-35 to +35 degrees)
    speed_mps: float  # Target speed (m/s)


class TrajectoryPlanner:
    """Trajectory planner and steering command generator"""

    def __init__(self):
        """Initialize trajectory planner"""
        self.waypoints: List[Waypoint] = []
        self.current_waypoint_index = 0
        self.Kp = 0.5  # PID proportional gain
        self.Ki = 0.1  # PID integral gain
        self.Kd = 0.2  # PID derivative gain
        self.error_integral = 0.0
        self.last_error = 0.0
        self.max_steering_angle = 35.0  # degrees

    def set_waypoints(self, waypoints: List[Waypoint]):
        """Set trajectory waypoints"""
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.error_integral = 0.0

    def calculate_cross_track_error(self, x: float, y: float, heading: float) -> float:
        """Calculate cross-track error from current path"""
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            return 0.0
        
        # Get current and next waypoints
        wp_current = self.waypoints[self.current_waypoint_index]
        
        # Vector from current position to waypoint
        dx = wp_current.x - x
        dy = wp_current.y - y
        
        # Cross-track error (perpendicular distance to desired path)
        # Using heading vector
        cos_h = np.cos(heading)
        sin_h = np.sin(heading)
        
        cte = dy * cos_h - dx * sin_h
        return cte

    def generate_steering_command(self, x: float, y: float, heading: float, 
                                 dt: float = 0.1) -> SteeringCommand:
        """Generate steering command using PID controller"""
        # Calculate cross-track error
        error = self.calculate_cross_track_error(x, y, heading)
        
        # PID controller
        self.error_integral += error * dt
        error_derivative = (error - self.last_error) / dt
        self.last_error = error
        
        # Steering angle
        steering_angle = (self.Kp * error + 
                         self.Ki * self.error_integral + 
                         self.Kd * error_derivative)
        
        # Clamp steering angle
        steering_angle = np.clip(steering_angle, 
                                -self.max_steering_angle, 
                                self.max_steering_angle)
        
        # Target speed (adaptive based on steering angle)
        speed = 5.0 * (1.0 - abs(steering_angle) / 90.0)
        
        return SteeringCommand(angle_deg=steering_angle, speed_mps=max(0.5, speed))

    def is_at_waypoint(self, x: float, y: float, tolerance: float = 1.0) -> bool:
        """Check if current position is at waypoint"""
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            return False
        
        wp = self.waypoints[self.current_waypoint_index]
        distance = np.sqrt((x - wp.x)**2 + (y - wp.y)**2)
        
        return distance <= tolerance

    def advance_waypoint(self):
        """Move to next waypoint"""
        if self.current_waypoint_index < len(self.waypoints) - 1:
            self.current_waypoint_index += 1

    def get_current_waypoint(self) -> Waypoint:
        """Get current target waypoint"""
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            return None
        return self.waypoints[self.current_waypoint_index]
