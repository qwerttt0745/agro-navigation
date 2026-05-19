import math
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class Waypoint:
    x: float
    y: float


class TractorModel:
    def __init__(self, x: float = 50.0, y: float = 50.0, heading: float = 0.0):
        self.x = x
        self.y = y
        self.heading = heading
        self.speed = 2.5
        self.wheel_angle = 0.0
        self.wheelbase = 2.8

        self.field_width = 500.0
        self.field_height = 300.0
        self.strip_width = 6.0

        self.waypoints = self._generate_waypoints()
        self.current_waypoint_idx = 0
        self.trajectory_history = [(self.x, self.y)]
        
    def _generate_waypoints(self) -> List[Waypoint]:
        waypoints = []
        num_strips = int(self.field_height / self.strip_width) + 1
        
        for i in range(num_strips):
            y = 50.0 + i * self.strip_width
            if y > 50.0 + self.field_height:
                break
            
            if i % 2 == 0:
                waypoints.append(Waypoint(50.0, y))
                waypoints.append(Waypoint(50.0 + self.field_width, y))
            else:
                waypoints.append(Waypoint(50.0 + self.field_width, y))
                waypoints.append(Waypoint(50.0, y))
        
        return waypoints
    
    def update(self, dt: float):
        self.x += self.speed * math.cos(self.heading) * dt
        self.y += self.speed * math.sin(self.heading) * dt

        self.heading += (self.speed / self.wheelbase) * math.tan(self.wheel_angle) * dt

        self.heading = math.atan2(math.sin(self.heading), math.cos(self.heading))

        self.trajectory_history.append((self.x, self.y))

        if len(self.trajectory_history) > 1000:
            self.trajectory_history = self.trajectory_history[-1000:]
    
    def get_target_heading(self) -> float:
        if self.current_waypoint_idx >= len(self.waypoints):
            self.current_waypoint_idx = 0
        
        wp = self.waypoints[self.current_waypoint_idx]
        dx = wp.x - self.x
        dy = wp.y - self.y
        target_heading = math.atan2(dy, dx)

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
        Kp_heading = 0.5
        Kd_cte = 0.3

        steering_command = Kp_heading * heading_error + Kd_cte * cross_track_error

        self.wheel_angle = max(-0.35, min(0.35, steering_command))
