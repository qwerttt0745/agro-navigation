"""
IMUSimulator: Simulates IMU (accelerometer + gyroscope) sensor data
"""
import math
import random
from dataclasses import dataclass


@dataclass
class IMUData:
    """IMU measurement data"""
    ax: float  # acceleration X (m/s²)
    ay: float  # acceleration Y (m/s²)
    gz: float  # angular velocity Z - gyroscope (rad/s)


class IMUSimulator:
    """
    Simulates 6-DOF IMU with realistic noise and bias
    """
    
    def __init__(self, update_rate: int = 100):
        """
        Args:
            update_rate: IMU update rate in Hz (default 100 Hz)
        """
        self.update_rate = update_rate
        self.dt = 1.0 / update_rate
        
        # Noise parameters
        self.accel_noise_std = 0.01  # m/s² sigma
        self.gyro_noise_std = 0.001  # rad/s sigma
        
        # Gyro bias (drifts over time)
        self.gyro_bias_z = 0.0
        self.gyro_bias_rate = 1e-5  # rad/s per second (bias instability)
        
        self.bias_instability = 4.85e-5  # 10°/hour in rad/s
        self._last_heading = None
    
    def get_reading(self, true_heading: float, true_speed: float, dt: float) -> IMUData:
        """
        Get IMU reading with realistic noise
        Args:
            true_heading: True tractor heading (radians)
            true_speed: True tractor speed (m/s)
            dt: Time step in seconds
        Returns:
            IMUData with noisy measurements
        """
        # Calculate expected accelerations based on kinematics
        # Simplified model: assuming constant speed, acceleration is mainly from turning
        ax = random.gauss(0, self.accel_noise_std)
        ay = random.gauss(0, self.accel_noise_std)
        
        # Gyro reading: angular velocity with noise and drift
        self.gyro_bias_z += random.gauss(0, self.gyro_bias_rate * dt)

        # Estimate true turn rate from heading change
        if self._last_heading is None or dt <= 0:
            true_gz = 0.0
        else:
            delta = math.atan2(
                math.sin(true_heading - self._last_heading),
                math.cos(true_heading - self._last_heading)
            )
            true_gz = delta / dt
        self._last_heading = true_heading

        gz = true_gz + random.gauss(0, self.gyro_noise_std) + self.gyro_bias_z
        
        return IMUData(ax=ax, ay=ay, gz=gz)
    
    def apply_low_pass_filter(self, current_value: float, new_value: float, alpha: float = 0.8) -> float:
        """
        Apply exponential low-pass filter to smooth noisy sensor readings
        Args:
            current_value: Previous filtered value
            new_value: New raw sensor value
            alpha: Filter coefficient (0-1, higher = more responsive)
        Returns:
            Filtered value
        """
        return alpha * new_value + (1 - alpha) * current_value
    
    def calibrate(self):
        """Calibrate IMU (reset biases) - stub for demonstration"""
        self.gyro_bias_z = 0.0
