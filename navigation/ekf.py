import numpy as np
import math
from dataclasses import dataclass


@dataclass
class KalmanState:
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    
    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.heading, self.vx, self.vy])
    
    @staticmethod
    def from_array(arr: np.ndarray) -> 'KalmanState':
        return KalmanState(
            x=float(arr[0]),
            y=float(arr[1]),
            heading=float(arr[2]),
            vx=float(arr[3]),
            vy=float(arr[4])
        )


class ExtendedKalmanFilter:
    def __init__(self):
        self.X = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.P = np.diag([1.0, 1.0, 0.1, 0.5, 0.5])
        self.Q = np.diag([0.01, 0.01, 0.001, 0.1, 0.1])
        self.R_gnss_rtk = np.diag([0.0004, 0.0004, 0.01])
        self.R_gnss_float = np.diag([0.022, 0.022, 0.01])
        self.R_gnss_single = np.diag([4.0, 4.0, 0.01])
        self.R_gnss = self.R_gnss_rtk
        self.R_imu = np.diag([0.0001, 0.01])
        self.R_lidar = np.array([[0.01]])
        self.last_update_time = 0.0
        self.is_initialized = False
        self.eps = 1e-9
    
    def predict(self, imu_data: dict, dt: float):
        if dt <= 0:
            return
        ax = imu_data.get('ax', 0.0)
        ay = imu_data.get('ay', 0.0)
        gz = imu_data.get('gz', 0.0)
        x, y, heading, vx, vy = self.X
        x_new = x + vx * dt
        y_new = y + vy * dt
        heading_new = heading + gz * dt
        vx_new = vx + ax * dt
        vy_new = vy + ay * dt
        heading_new = math.atan2(math.sin(heading_new), math.cos(heading_new))
        X_pred = np.array([x_new, y_new, heading_new, vx_new, vy_new])
        F = np.eye(5)
        F[0, 3] = dt
        F[1, 4] = dt
        self.P = F @ self.P @ F.T + self.Q
        np.fill_diagonal(self.P, np.maximum(np.diag(self.P), self.eps))
        self.X = X_pred
    
    def update_gnss(self, gnss_data: dict):
        if gnss_data.get('is_fixed') is False:
            return
        if 'lat' in gnss_data and 'lon' in gnss_data:
            x_meas = gnss_data.get('x', gnss_data.get('local_x', 0.0))
            y_meas = gnss_data.get('y', gnss_data.get('local_y', 0.0))
        else:
            x_meas = gnss_data.get('x', 0.0)
            y_meas = gnss_data.get('y', 0.0)
        heading_meas = gnss_data.get('heading', self.X[2])
        z = np.array([x_meas, y_meas, heading_meas])
        H = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 1, 0, 0]
        ])
        mode = gnss_data.get('mode', 'RTK_FIXED')
        if 'RTK_FIXED' in str(mode):
            self.R_gnss = self.R_gnss_rtk
        elif 'RTK_FLOAT' in str(mode):
            self.R_gnss = self.R_gnss_float
        else:
            self.R_gnss = self.R_gnss_single
        y = z - (H @ self.X)
        S = H @ self.P @ H.T + self.R_gnss
        try:
            K = self.P @ H.T @ np.linalg.inv(S + np.eye(3) * self.eps)
        except:
            return
        self.X = self.X + (K @ y)
        self.P = (np.eye(5) - K @ H) @ self.P
        np.fill_diagonal(self.P, np.maximum(np.diag(self.P), self.eps))
        self.X[2] = math.atan2(math.sin(self.X[2]), math.cos(self.X[2]))
        self.is_initialized = True

    def get_accuracy_estimate(self) -> dict:
        state = self.get_state()
        pos_uncertainty = float(state.get('position_uncertainty', 1.0))

        return {
            'position_rmse_m': round(pos_uncertainty, 4),
            'heading_rmse_deg': round(math.degrees(pos_uncertainty * 0.1), 3),
            'confidence': round(max(0.0, 1.0 - pos_uncertainty / 5.0), 3)
        }
    
    def update_lidar(self, lidar_correction: tuple):
        dx, dy = lidar_correction
        z = np.array([dy])
        H = np.array([[0, 1, 0, 0, 0]])
        y = z - (H @ self.X)
        S = H @ self.P @ H.T + self.R_lidar
        try:
            K = self.P @ H.T / (S[0, 0] + self.eps)
            K = K.reshape(-1, 1)
        except:
            return
        self.X = self.X + K.flatten() * y[0]
        self.P = (np.eye(5) - K @ H) @ self.P
        np.fill_diagonal(self.P, np.maximum(np.diag(self.P), self.eps))
    
    def get_state(self) -> dict:
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
    
    def get_state_object(self) -> KalmanState:
        return KalmanState.from_array(self.X)
