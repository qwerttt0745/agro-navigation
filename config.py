"""
Конфігурація системи навігації агротехніки
Всі параметри в одному місці — легко змінювати для тестування
"""
from dataclasses import dataclass


@dataclass
class Settings:
    # Server
    HOST: str = "0.0.0.0"
    PORT: int = 8000

    # Simulation
    DT: float = 0.1  # seconds (10 Hz)

    # Field
    FIELD_WIDTH: float = 500.0
    FIELD_HEIGHT: float = 300.0
    STRIP_WIDTH: float = 6.0

    # Vehicle
    VEHICLE_SPEED: float = 2.5  # m/s
    VEHICLE_WHEELBASE: float = 2.8

    # GNSS
    BASE_LAT: float = 47.4100
    BASE_LON: float = 35.9180
    RTK_ACCURACY: float = 0.02
    SNR_THRESHOLD: float = 35.0

    # Navigation modes
    DR_ACTIVATION_DELAY: float = 0.0
    LIDAR_ACTIVATION_DELAY: float = 30.0
    SAFE_STOP_DELAY: float = 120.0

    # Thresholds
    MAX_DR_ERROR: float = 0.30

    # IMU
    IMU_FREQUENCY: float = 100.0
    IMU_GYRO_NOISE: float = 0.001
    IMU_ACCEL_NOISE: float = 0.01
    IMU_DRIFT_RATE: float = 0.0001


settings = Settings()
