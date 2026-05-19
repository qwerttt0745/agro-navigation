"""
TractorModel: Simulates a tractor following a planned route
Uses bicycle kinematic model with Pure Pursuit autopilot
"""
import math
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class Waypoint:
    """A waypoint on the field"""
    x: float
    y: float


class TractorModel:
    """
    Simulates a tractor moving autonomously across a field.
    Uses pure rectangular field with zigzag path (parallel strips).
    """
    
    def __init__(self, x: float = 50.0, y: float = 50.0, heading: float = 0.0):
        """
        Args:
            x, y: initial position in meters (local coordinates)
            heading: initial heading in radians (0 = North)
        """
        self.x = x
        self.y = y
        self.heading = heading
        self.speed = 2.5  # m/s (9 km/h - realistic for field work)
        self.wheel_angle = 0.0  # steering angle (radians)
        self.wheelbase = 2.8  # meters
        
        # Field dimensions (500 x 300 meters)
        self.field_width = 500.0
        self.field_height = 300.0
        self.strip_width = 6.0  # swath width in meters
        
        # Generate zigzag trajectory
        self.waypoints = self._generate_waypoints()
        self.current_waypoint_idx = 0
        self.trajectory_history = [(self.x, self.y)]
        
    def _generate_waypoints(self) -> List[Waypoint]:
        """Generate zigzag parallel strip waypoints"""
        waypoints = []
        num_strips = int(self.field_height / self.strip_width) + 1
        
        for i in range(num_strips):
            y = 50.0 + i * self.strip_width
            if y > 50.0 + self.field_height:
                break
            
            if i % 2 == 0:
                # Going right
                waypoints.append(Waypoint(50.0, y))
                waypoints.append(Waypoint(50.0 + self.field_width, y))
            else:
                # Going left
                waypoints.append(Waypoint(50.0 + self.field_width, y))
                waypoints.append(Waypoint(50.0, y))
        
        return waypoints
    
    def update(self, dt: float):
        """
        Update tractor position using bicycle kinematic model
        Args:
            dt: time step in seconds
        """
        # Bicycle model kinematics
        self.x += self.speed * math.cos(self.heading) * dt
        self.y += self.speed * math.sin(self.heading) * dt
        
        # Update heading based on steering angle
        self.heading += (self.speed / self.wheelbase) * math.tan(self.wheel_angle) * dt
        
        # Normalize heading to [-pi, pi]
        self.heading = math.atan2(math.sin(self.heading), math.cos(self.heading))
        
        self.trajectory_history.append((self.x, self.y))
        
        # Keep track history size reasonable
        if len(self.trajectory_history) > 1000:
            self.trajectory_history = self.trajectory_history[-1000:]
    
    def get_target_heading(self) -> float:
        """Get target heading to next waypoint"""
        if self.current_waypoint_idx >= len(self.waypoints):
            self.current_waypoint_idx = 0
        
        wp = self.waypoints[self.current_waypoint_idx]
        dx = wp.x - self.x
        dy = wp.y - self.y
        target_heading = math.atan2(dy, dx)
        
        # Check if reached waypoint (within 5 meters)
        distance = math.sqrt(dx**2 + dy**2)
        if distance < 5.0:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.waypoints):
                self.current_waypoint_idx = 0
            wp = self.waypoints[self.current_waypoint_idx]
            dx = wp.x - self.x
            dy = wp.y - self.y
            target_heading = math.atan2(dy, dx)
        
        return target_heading
    
    def apply_autopilot(self, cross_track_error: float, heading_error: float):
        """
        Simple Pure Pursuit controller
        Args:
            cross_track_error: lateral deviation from path (m)
            heading_error: heading error (radians)
        """
        # PID parameters
        Kp_heading = 0.5
        Kd_cte = 0.3
        
        # Steering command
        steering_command = Kp_heading * heading_error + Kd_cte * cross_track_error
        
        # Limit steering angle to ±0.35 radians (~20 degrees)
        self.wheel_angle = max(-0.35, min(0.35, steering_command))
