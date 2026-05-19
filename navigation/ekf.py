"""
Extended Kalman Filter (EKF) for multi-sensor fusion
Fuses GNSS, IMU, and LiDAR measurements for optimal position estimation
"""
import numpy as np
import math
from dataclasses import dataclass


@dataclass
class KalmanState:
    """State vector"""
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    
    def to_array(self) -> np.ndarray:
        """Convert to numpy array"""
        return np.array([self.x, self.y, self.heading, self.vx, self.vy])
    
    @staticmethod
    def from_array(arr: np.ndarray) -> 'KalmanState':
        """Create from numpy array"""
        return KalmanState(
            x=float(arr[0]),
            y=float(arr[1]),
            heading=float(arr[2]),
            vx=float(arr[3]),
            vy=float(arr[4])
        )


class ExtendedKalmanFilter:
    """
    Розширений фільтр Калмана (EKF) для комплексування сенсорних даних.

    Реалізує алгоритм Sensor Fusion відповідно до вимоги FR-03 курсової роботи.

    Вектор стану X = [x, y, heading, vx, vy]
    де:
        x, y     — позиція у метрах (локальна система координат)
        heading  — курс у радіанах
        vx, vy   — компоненти швидкості, м/с

    Джерела вимірювань:
        - GNSS/RTK: абсолютна позиція (x, y), похибка σ = 0.02 м
        - IMU: кутова швидкість, прискорення — для предикції
        - LiDAR: корекція відносно рядків культур

    Частота оновлення: 10 Гц (dt = 0.1 с) — відповідає NFR-PER-03
    """
    
    def __init__(self):
        """Initialize EKF"""
        # State [x, y, heading, vx, vy]
        self.X = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Covariance matrix P (5x5)
        self.P = np.diag([1.0, 1.0, 0.1, 0.5, 0.5])
        
        # Process noise Q (5x5)
        self.Q = np.diag([0.01, 0.01, 0.001, 0.1, 0.1])
        
        # GNSS measurement noise (3x3) for [x, y, heading]
        self.R_gnss_rtk = np.diag([0.0004, 0.0004, 0.01])  # RTK Fixed
        self.R_gnss_float = np.diag([0.022, 0.022, 0.01])  # RTK Float (15cm)
        self.R_gnss_single = np.diag([4.0, 4.0, 0.01])     # Single (2m noise)
        self.R_gnss = self.R_gnss_rtk
        
        # IMU measurement noise (2x2) for [ax, ay]
        self.R_imu = np.diag([0.0001, 0.01])
        
        # LiDAR measurement noise (1x1)
        self.R_lidar = np.array([[0.01]])
        
        # Time tracking
        self.last_update_time = 0.0
        self.is_initialized = False
        
        # Small constant to avoid singularities
        self.eps = 1e-9
    
    def predict(self, imu_data: dict, dt: float):
        """
        Prediction step using IMU data
        Args:
            imu_data: Dictionary with 'ax', 'ay', 'gz' keys
            dt: Time step in seconds
        """
        if dt <= 0:
            return
        
        # Extract IMU readings
        ax = imu_data.get('ax', 0.0)
        ay = imu_data.get('ay', 0.0)
        gz = imu_data.get('gz', 0.0)
        
        # Unpack state
        x, y, heading, vx, vy = self.X
        
        # Kinematic model
        # Position update based on velocity
        x_new = x + vx * dt
        y_new = y + vy * dt
        
        # Heading update based on gyro
        heading_new = heading + gz * dt
        
        # Velocity update based on acceleration
        vx_new = vx + ax * dt
        vy_new = vy + ay * dt
        
        # Normalize heading to [-pi, pi]
        heading_new = math.atan2(math.sin(heading_new), math.cos(heading_new))
        
        # Update state
        X_pred = np.array([x_new, y_new, heading_new, vx_new, vy_new])
        
        # Calculate Jacobian F for linear approximation
        # F is mostly identity, except for the kinematic couplings
        F = np.eye(5)
        # Position affected by velocity
        F[0, 3] = dt  # dx/dvx
        F[1, 4] = dt  # dy/dvy
        # Velocity affected by acceleration (linear model, no explicit coupling)
        
        # Update covariance: P = F @ P @ F.T + Q
        self.P = F @ self.P @ F.T + self.Q
        
        # Clamp diag to prevent numerical issues
        np.fill_diagonal(self.P, np.maximum(np.diag(self.P), self.eps))
        
        self.X = X_pred
    
    def update_gnss(self, gnss_data: dict):
        """
        GNSS measurement update step
        Args:
            gnss_data: Dictionary with 'x', 'y', 'heading' keys (or 'lat', 'lon', 'alt')
        """
        if gnss_data.get('is_fixed') is False:
            return
        # Convert lat/lon to local if needed
        if 'lat' in gnss_data and 'lon' in gnss_data:
            x_meas = gnss_data.get('x', gnss_data.get('local_x', 0.0))
            y_meas = gnss_data.get('y', gnss_data.get('local_y', 0.0))
        else:
            x_meas = gnss_data.get('x', 0.0)
            y_meas = gnss_data.get('y', 0.0)
        
        heading_meas = gnss_data.get('heading', self.X[2])
        
        # Measurement vector and matrix
        z = np.array([x_meas, y_meas, heading_meas])
        H = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 1, 0, 0]
        ])
        
        # Update R based on signal quality
        mode = gnss_data.get('mode', 'RTK_FIXED')
        if 'RTK_FIXED' in str(mode):
            self.R_gnss = self.R_gnss_rtk
        elif 'RTK_FLOAT' in str(mode):
            self.R_gnss = self.R_gnss_float
        else:
            self.R_gnss = self.R_gnss_single
        
        # Innovation
        y = z - (H @ self.X)
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_gnss
        
        # Kalman gain
        try:
            K = self.P @ H.T @ np.linalg.inv(S + np.eye(3) * self.eps)
        except:
            return  # Skip update if singular
        
        # Update state
        self.X = self.X + (K @ y)
        
        # Update covariance
        self.P = (np.eye(5) - K @ H) @ self.P
        
        # Clamp diag to prevent numerical issues
        np.fill_diagonal(self.P, np.maximum(np.diag(self.P), self.eps))
        
        # Normalize heading
        self.X[2] = math.atan2(math.sin(self.X[2]), math.cos(self.X[2]))
        
        self.is_initialized = True

    def get_accuracy_estimate(self) -> dict:
        """
        Повертає поточну оцінку точності позиціонування.
        Використовується для верифікації вимог NFR-PER-01 та NFR-PER-02.

        Returns:
            dict з ключами:
                position_rmse_m  — оцінка похибки позиції в метрах
                heading_rmse_deg — оцінка похибки курсу в градусах
                confidence       — рівень довіри (0.0 - 1.0)
        """
        state = self.get_state()
        pos_uncertainty = float(state.get('position_uncertainty', 1.0))

        return {
            'position_rmse_m': round(pos_uncertainty, 4),
            'heading_rmse_deg': round(math.degrees(pos_uncertainty * 0.1), 3),
            'confidence': round(max(0.0, 1.0 - pos_uncertainty / 5.0), 3)
        }
    
    def update_lidar(self, lidar_correction: tuple):
        """
        LiDAR measurement update step
        Args:
            lidar_correction: (dx, dy) correction tuple from LiDAR SLAM
        """
        dx, dy = lidar_correction
        
        # LiDAR provides mainly Y (lateral) correction
        z = np.array([dy])
        H = np.array([[0, 1, 0, 0, 0]])
        
        # Innovation
        y = z - (H @ self.X)
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_lidar
        
        # Kalman gain
        try:
            K = self.P @ H.T / (S[0, 0] + self.eps)
            K = K.reshape(-1, 1)
        except:
            return
        
        # Update state
        self.X = self.X + K.flatten() * y[0]
        
        # Update covariance
        self.P = (np.eye(5) - K @ H) @ self.P
        
        # Clamp diag to prevent numerical issues
        np.fill_diagonal(self.P, np.maximum(np.diag(self.P), self.eps))
    
    def get_state(self) -> dict:
        """Get current state as dictionary"""
        state_dict = {
            'x': float(self.X[0]),
            'y': float(self.X[1]),
            'heading': float(self.X[2]),
            'vx': float(self.X[3]),
            'vy': float(self.X[4]),
            'position_uncertainty': float(np.sqrt(self.P[0, 0] + self.P[1, 1])),
            'heading_uncertainty': float(np.sqrt(self.P[2, 2]))
        }
        return state_dict

    def get_accuracy_estimate(self) -> dict:
        """
        Return current positioning accuracy estimate.

        Returns:
            dict with keys:
                position_rmse_m  - position error estimate in meters
                heading_rmse_deg - heading error estimate in degrees
                confidence       - confidence level (0.0 to 1.0)
        """
        state = self.get_state()
        pos_uncertainty = float(state.get('position_uncertainty', 1.0))

        return {
            'position_rmse_m': round(pos_uncertainty, 4),
            'heading_rmse_deg': round(math.degrees(pos_uncertainty * 0.1), 3),
            'confidence': round(max(0.0, 1.0 - pos_uncertainty / 5.0), 3)
        }
    
    def get_state_object(self) -> KalmanState:
        """Get current state as KalmanState object"""
        return KalmanState.from_array(self.X)
