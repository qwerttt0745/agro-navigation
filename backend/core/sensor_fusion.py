"""Extended Kalman Filter for sensor fusion"""
import numpy as np
from typing import Tuple
from .state_vector import StateVector, SourceEnum


class SensorFusionUnit:
    """Extended Kalman Filter for multi-sensor fusion"""

    def __init__(self):
        """Initialize EKF with 9-dimensional state"""
        self.dt = 0.1  # 10 Hz cycle
        self.initialized = False
        
        # State: [x, y, z, heading, vx, vy, roll, pitch, yaw_rate]
        self.x = np.array([0.0, 0.0, 120.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Covariance matrix (9x9)
        self.P = np.eye(9) * 1000.0
        self.P[0, 0] = self.P[1, 1] = 100.0  # Position uncertainty
        self.P[2, 2] = 10.0  # Height uncertainty
        
        # Process noise (Q matrix)
        self.Q = np.eye(9) * 0.01
        self.Q[6:9, 6:9] *= 0.1  # Lower process noise for angles
        
        # Measurement noise for GNSS
        self.R_gnss = np.eye(3) * 0.0004  # ±2cm accuracy
        
        # Measurement noise for LiDAR
        self.R_lidar = np.eye(2) * 0.01
        
        # Chi-squared threshold for outlier rejection (95% confidence)
        self.chi2_threshold = 7.815

    def predict(self, imu_data: dict) -> StateVector:
        """Prediction step using IMU data"""
        # Extract IMU data
        accel_x = imu_data.get('accel_x', 0.0)
        accel_y = imu_data.get('accel_y', 0.0)
        accel_z = imu_data.get('accel_z', 0.0)
        gyro_x = imu_data.get('gyro_x', 0.0)
        gyro_y = imu_data.get('gyro_y', 0.0)
        gyro_z = imu_data.get('gyro_z', 0.0)
        
        # Update state using IMU
        heading = self.x[3]
        cos_h = np.cos(heading)
        sin_h = np.sin(heading)
        
        # Rotate accelerations to navigation frame
        ax_nav = accel_x * cos_h - accel_y * sin_h
        ay_nav = accel_x * sin_h + accel_y * cos_h
        
        # Update velocities
        self.x[4] += ax_nav * self.dt
        self.x[5] += ay_nav * self.dt
        
        # Update position
        self.x[0] += self.x[4] * self.dt
        self.x[1] += self.x[5] * self.dt
        self.x[2] += accel_z * self.dt
        
        # Update heading
        self.x[3] += gyro_z * self.dt
        
        # Update roll/pitch
        self.x[6] += gyro_x * self.dt
        self.x[7] += gyro_y * self.dt
        self.x[8] = gyro_z
        
        # Update covariance
        F = self._compute_jacobian()
        self.P = F @ self.P @ F.T + self.Q
        
        # Accumulate position uncertainty
        self.P[0, 0] += 0.01
        self.P[1, 1] += 0.01
        
        return self._state_to_vector(SourceEnum.FUSION)

    def update_gnss(self, gnss_data: dict) -> StateVector:
        """Update with GNSS measurement"""
        if not gnss_data.get('is_fixed', False):
            return self._state_to_vector(SourceEnum.GNSS)
        
        # GNSS measurement: [x, y, z]
        z = np.array([
            gnss_data.get('x', 0.0),
            gnss_data.get('y', 0.0),
            gnss_data.get('z', 120.0)
        ])
        
        # Innovation
        y = z - self.x[:3]
        
        # Measurement jacobian (partial of measurement against state)
        H = np.zeros((3, 9))
        H[0, 0] = H[1, 1] = H[2, 2] = 1.0
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_gnss
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Outlier rejection using chi-squared test
        if self.initialized:
            chi2 = y.T @ np.linalg.inv(S) @ y
            if chi2 > self.chi2_threshold:
                return self._state_to_vector(SourceEnum.GNSS)  # Reject outlier
        else:
            self.initialized = True
        
        # Update state (K is 9x3, y is 3, so K @ y is 9)
        self.x += K @ y
        
        # Update covariance
        self.P = (np.eye(9) - K @ H) @ self.P
        
        return self._state_to_vector(SourceEnum.GNSS)

    def update_lidar(self, lidar_correction: Tuple[float, float]) -> StateVector:
        """Update with LiDAR SLAM correction"""
        dx, dy = lidar_correction
        
        # LiDAR measurement
        z = np.array([dx, dy])
        
        # Current position offset
        y = z - np.array([self.x[0], self.x[1]])
        
        # Measurement jacobian
        H = np.zeros((2, 9))
        H[0, 0] = H[1, 1] = 1.0
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_lidar
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.x[:2] += K @ y
        
        # Update covariance
        self.P = (np.eye(9) - K @ H) @ self.P
        
        return self._state_to_vector(SourceEnum.LIDAR)

    def update_odometry(self, distance: float, heading_change: float) -> StateVector:
        """Update with odometry data"""
        heading = self.x[3]
        
        # Dead reckoning update
        self.x[0] += distance * np.sin(heading)
        self.x[1] += distance * np.cos(heading)
        self.x[3] += heading_change
        
        # Increase uncertainty
        self.P[0, 0] += 0.01 * distance
        self.P[1, 1] += 0.01 * distance
        
        return self._state_to_vector(SourceEnum.DEAD_RECKONING)

    def _compute_jacobian(self) -> np.ndarray:
        """Compute Jacobian matrix F"""
        F = np.eye(9)
        heading = self.x[3]
        cos_h = np.cos(heading)
        sin_h = np.sin(heading)
        
        # Position derivatives
        F[0, 4] = self.dt * cos_h
        F[1, 5] = self.dt * sin_h
        
        return F

    def _state_to_vector(self, source: SourceEnum) -> StateVector:
        """Convert internal state to StateVector"""
        return StateVector(
            x=float(self.x[0]),
            y=float(self.x[1]),
            z=float(self.x[2]),
            heading=float(self.x[3]),
            velocity_x=float(self.x[4]),
            velocity_y=float(self.x[5]),
            roll=float(self.x[6]),
            pitch=float(self.x[7]),
            yaw_rate=float(self.x[8]),
            position_uncertainty=float(np.sqrt(self.P[0, 0])),
            source=source
        )
