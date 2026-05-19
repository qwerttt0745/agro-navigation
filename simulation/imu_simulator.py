import math
import random
from dataclasses import dataclass


@dataclass
class IMUData:
    ax: float
    ay: float
    gz: float


class IMUSimulator:
    def __init__(self, update_rate: int = 100):
        self.update_rate = update_rate
        self.dt = 1.0 / update_rate

        self.accel_noise_std = 0.01
        self.gyro_noise_std = 0.001

        self.gyro_bias_z = 0.0
        self.gyro_bias_rate = 1e-5

        self.bias_instability = 4.85e-5
        self._last_heading = None
    
    def get_reading(self, true_heading: float, true_speed: float, dt: float) -> IMUData:
        ax = random.gauss(0, self.accel_noise_std)
        ay = random.gauss(0, self.accel_noise_std)

        self.gyro_bias_z += random.gauss(0, self.gyro_bias_rate * dt)

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
        return alpha * new_value + (1 - alpha) * current_value
    
    def calibrate(self):
        self.gyro_bias_z = 0.0
