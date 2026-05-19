import math


class DeadReckoningModule:
    def __init__(self):
        self.last_known_x = 0.0
        self.last_known_y = 0.0
        self.last_known_heading = 0.0
        
        self.estimated_x = 0.0
        self.estimated_y = 0.0
        self.estimated_heading = 0.0
        
        self.drift_error = 0.0
        self.distance_traveled = 0.0
        self._distance_traveled = 0.0
        
        self.active = False
    
    def activate(self, x: float, y: float, heading: float):
        self.last_known_x = x
        self.last_known_y = y
        self.last_known_heading = heading
        
        self.estimated_x = x
        self.estimated_y = y
        self.estimated_heading = heading
        
        self.drift_error = 0.0
        self.distance_traveled = 0.0
        self._distance_traveled = 0.0
        self.active = True
    
    def deactivate(self):
        self.active = False
    
    def update(self, imu_data: dict, wheel_speed: float, dt: float, true_x: float = None, true_y: float = None):
        if not self.active or dt <= 0:
            return
        
        gz = imu_data.get('gz', 0.0)
        
        self.estimated_heading += gz * dt
        self.estimated_heading = self._normalize_heading(self.estimated_heading)
        
        dx = wheel_speed * math.cos(self.estimated_heading) * dt
        dy = wheel_speed * math.sin(self.estimated_heading) * dt
        
        self.estimated_x += dx
        self.estimated_y += dy
        
        distance_step = math.sqrt(dx**2 + dy**2)
        self.distance_traveled += distance_step
        self._distance_traveled += distance_step
        
        if true_x is not None and true_y is not None:
            error_x = self.estimated_x - true_x
            error_y = self.estimated_y - true_y
            self.drift_error = math.sqrt(error_x**2 + error_y**2)
    
    def reset(self, x: float, y: float, heading: float):
        self.estimated_x = x
        self.estimated_y = y
        self.estimated_heading = heading
        self.drift_error = 0.0
        self.distance_traveled = 0.0
        self._distance_traveled = 0.0
    
    def get_position(self) -> tuple:
        return (self.estimated_x, self.estimated_y, self.estimated_heading)
    
    def get_drift_error(self) -> float:
        if not hasattr(self, '_distance_traveled'):
            self._distance_traveled = 0.0
        return min(self._distance_traveled * 0.0025, 5.0)
    
    def _normalize_heading(self, heading: float) -> float:
        return math.atan2(math.sin(heading), math.cos(heading))
