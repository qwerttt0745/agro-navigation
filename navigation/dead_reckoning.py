"""
Dead Reckoning: Position estimation without GNSS using IMU integration
"""
import math


class DeadReckoningModule:
    """
    Dead Reckoning module for position estimation during GNSS loss
    Integrates IMU measurements to track position changes
    """
    
    def __init__(self):
        """Initialize dead reckoning"""
        self.last_known_x = 0.0
        self.last_known_y = 0.0
        self.last_known_heading = 0.0
        
        self.estimated_x = 0.0
        self.estimated_y = 0.0
        self.estimated_heading = 0.0
        
        self.drift_error = 0.0  # Accumulated position error
        self.distance_traveled = 0.0
        
        self.active = False
    
    def activate(self, x: float, y: float, heading: float):
        """
        Activate dead reckoning from a known position
        Args:
            x, y: Known position in meters
            heading: Known heading in radians
        """
        self.last_known_x = x
        self.last_known_y = y
        self.last_known_heading = heading
        
        self.estimated_x = x
        self.estimated_y = y
        self.estimated_heading = heading
        
        self.drift_error = 0.0
        self.distance_traveled = 0.0
        self.active = True
    
    def deactivate(self):
        """Deactivate dead reckoning"""
        self.active = False
    
    def update(self, imu_data: dict, wheel_speed: float, dt: float, true_x: float = None, true_y: float = None):
        """
        Update dead reckoning position using IMU data
        Args:
            imu_data: Dictionary with 'ax', 'ay', 'gz' keys
            wheel_speed: Wheel speed from odometer (m/s)
            dt: Time step in seconds
            true_x, true_y: True position (for drift error calculation)
        """
        if not self.active or dt <= 0:
            return
        
        # Extract gyro reading
        gz = imu_data.get('gz', 0.0)
        
        # Update heading based on gyro integration
        self.estimated_heading += gz * dt
        self.estimated_heading = self._normalize_heading(self.estimated_heading)
        
        # Update position based on wheel speed and heading
        # Using simplified model: movement in heading direction
        dx = wheel_speed * math.cos(self.estimated_heading) * dt
        dy = wheel_speed * math.sin(self.estimated_heading) * dt
        
        self.estimated_x += dx
        self.estimated_y += dy
        
        # Update distance traveled
        distance_step = math.sqrt(dx**2 + dy**2)
        self.distance_traveled += distance_step
        
        # Update drift error if true position is provided
        if true_x is not None and true_y is not None:
            error_x = self.estimated_x - true_x
            error_y = self.estimated_y - true_y
            self.drift_error = math.sqrt(error_x**2 + error_y**2)
    
    def reset(self, x: float, y: float, heading: float):
        """
        Reset dead reckoning to a new known position
        Clears accumulated drift error
        Args:
            x, y: New known position
            heading: New known heading
        """
        self.estimated_x = x
        self.estimated_y = y
        self.estimated_heading = heading
        self.drift_error = 0.0
        self.distance_traveled = 0.0
    
    def get_position(self) -> tuple:
        """
        Get current estimated position
        Returns:
            (x, y, heading) tuple
        """
        return (self.estimated_x, self.estimated_y, self.estimated_heading)
    
    def get_drift_error(self) -> float:
        """Get accumulated position error in meters"""
        return self.drift_error
    
    def _normalize_heading(self, heading: float) -> float:
        """Normalize heading to [-pi, pi] range"""
        return math.atan2(math.sin(heading), math.cos(heading))
