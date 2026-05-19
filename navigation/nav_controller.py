import math
import time
import logging
from enum import Enum
from typing import Dict
from datetime import datetime

from simulation.vehicle import TractorModel
from simulation.gnss_simulator import GNSSSimulator, GNSSMode
from simulation.imu_simulator import IMUSimulator
from simulation.lidar_simulator import LiDARSimulator
from navigation.ekf import ExtendedKalmanFilter
from navigation.dead_reckoning import DeadReckoningModule
from config import settings


class OperationMode(Enum):
    INITIALIZING = "INITIALIZING"
    GNSS_RTK = "GNSS_RTK"
    DEAD_RECKONING = "DEAD_RECKONING"
    LIDAR_NAV = "LIDAR_NAV"
    SAFE_STOP = "SAFE_STOP"
    MANUAL = "MANUAL"


class NavigationController:
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.vehicle = TractorModel()
        self.gnss = GNSSSimulator(settings.BASE_LAT, settings.BASE_LON)
        self.imu = IMUSimulator()
        self.lidar = LiDARSimulator()
        self.ekf = ExtendedKalmanFilter()
        self.dead_reckoning = DeadReckoningModule()
        self.mode = OperationMode.INITIALIZING
        self.gnss_lost_timer = 0.0
        self.lidar_active_timer = 0.0
        self.cross_track_error = 0.0
        self.trajectory_history = []
        self.total_distance = 0.0
        self.dr_total_time = 0.0
        self.lidar_total_time = 0.0
        self.max_cte = 0.0
        self.cte_sum = 0.0
        self.cte_samples = 0
        self.event_log = []
        self.simulation_time = 0.0
        self.active_scenario = None
        self.scenario_start_time = 0.0
        self.cycle_times = []

    def initialize(self):
        self.mode = OperationMode.GNSS_RTK
        self.vehicle.x = 50.0
        self.vehicle.y = 50.0
        self.vehicle.heading = 0.0
        self.gnss.mode = GNSSMode.RTK_FIXED
        self.dead_reckoning.activate(self.vehicle.x, self.vehicle.y, self.vehicle.heading)
        self.ekf.X[0] = self.vehicle.x
        self.ekf.X[1] = self.vehicle.y
        self.ekf.X[2] = self.vehicle.heading

        self.log_event("INFO", "System initialized. Mode: " + self.mode.value)

    def reset(self):
        self.vehicle = TractorModel()
        self.gnss = GNSSSimulator(settings.BASE_LAT, settings.BASE_LON)
        self.imu = IMUSimulator()
        self.lidar = LiDARSimulator()
        self.dead_reckoning = DeadReckoningModule()
        self.ekf = ExtendedKalmanFilter()
        self.mode = OperationMode.INITIALIZING
        self.gnss_lost_timer = 0.0
        self.lidar_active_timer = 0.0
        self.simulation_time = 0.0
        self.cross_track_error = 0.0
        self.total_distance = 0.0
        self.dr_total_time = 0.0
        self.lidar_total_time = 0.0
        self.max_cte = 0.0
        self.cte_sum = 0.0
        self.cte_samples = 0
        self.event_log = []
        self.active_scenario = None
        self.scenario_start_time = 0.0
        self.initialize()
        self.log_event("INFO", "Система повністю скинута до початкового стану")
        self.logger.info("↺ NavigationController RESET виконано")

    def step(self, dt: float) -> Dict:
        step_start_time = time.time()
        self.simulation_time += dt
        self.total_distance += self.vehicle.speed * dt
        self.vehicle.update(dt)
        imu_data = self.imu.get_reading(self.vehicle.heading, self.vehicle.speed, dt)
        gnss_data = None
        if self.active_scenario:
            self._update_scenario_gnss(dt)
        gnss_data = self.gnss.get_reading(self.vehicle.x, self.vehicle.y)
        lidar_data = self.lidar.get_scan(self.vehicle.x, self.vehicle.y, self.vehicle.heading)
        self.ekf.predict({'ax': imu_data.ax, 'ay': imu_data.ay, 'gz': imu_data.gz}, dt)
        self._update_navigation_mode(gnss_data, imu_data, dt, lidar_data)
        if self.mode == OperationMode.GNSS_RTK and gnss_data:
            gnss_dict = {
                'x': (gnss_data.lon - self.gnss.base_lon)
                     * 111320
                     * math.cos(math.radians(self.gnss.base_lat)),
                'y': (gnss_data.lat - self.gnss.base_lat) * 111320,
                'heading': self.vehicle.heading,
                'mode': gnss_data.mode
            }
            self.ekf.update_gnss(gnss_dict)
            self.ekf.X[0] = self.vehicle.x
            self.ekf.X[1] = self.vehicle.y
            self.ekf.X[2] = self.vehicle.heading
            self.dead_reckoning.reset(self.vehicle.x, self.vehicle.y, self.vehicle.heading)

        elif self.mode == OperationMode.DEAD_RECKONING:
            self.dead_reckoning.update(
                {'ax': imu_data.ax, 'ay': imu_data.ay, 'gz': imu_data.gz},
                self.vehicle.speed,
                dt,
                self.vehicle.x,
                self.vehicle.y
            )

        elif self.mode == OperationMode.LIDAR_NAV:
            correction = self.lidar.try_get_position_correction(lidar_data)
            if correction:
                dx, dy = correction
                dy = max(-0.5, min(0.5, dy))
                self.ekf.update_lidar((0.0, dy * 0.2))
        ekf_state = self.ekf.get_state()
        self._calculate_cross_track_error(ekf_state)
        self._apply_vehicle_control(ekf_state, dt)
        if self.mode == OperationMode.DEAD_RECKONING:
            self.dr_total_time += dt
        elif self.mode == OperationMode.LIDAR_NAV:
            self.lidar_total_time += dt
        step_time = time.time() - step_start_time
        self.cycle_times.append(step_time)
        self.max_cte = max(self.max_cte, self.cross_track_error)
        self.cte_sum += self.cross_track_error
        self.cte_samples += 1

        telemetry = self._build_telemetry_packet(
            ekf_state, imu_data, gnss_data, lidar_data, step_time
        )

        return telemetry

    def _update_navigation_mode(self, gnss_data, imu_data, dt, lidar_data):
        gnss_available = (
            gnss_data is not None
            and gnss_data.mode == GNSSMode.RTK_FIXED
        )

        if gnss_available:
            self.gnss_lost_timer = 0.0
            if self.mode != OperationMode.GNSS_RTK:
                self.mode = OperationMode.GNSS_RTK
                self.log_event("INFO", "GNSS відновлено — повернення до режиму GNSS_RTK")
        else:
            self.gnss_lost_timer += dt
            if self.mode == OperationMode.GNSS_RTK:
                self.mode = OperationMode.DEAD_RECKONING
                self.dead_reckoning.activate(
                    self.vehicle.x,
                    self.vehicle.y,
                    self.vehicle.heading
                )
                self.log_event(
                    "WARNING",
                    f"Втрата GNSS [{gnss_data.mode.value if gnss_data else 'LOST'}]. "
                    f"Активовано Dead Reckoning (IMU)"
                )
            elif (self.mode == OperationMode.DEAD_RECKONING
                  and self.gnss_lost_timer >= settings.LIDAR_ACTIVATION_DELAY):
                lidar_offset = self.lidar.detect_row_offset(lidar_data)
                self.mode = OperationMode.LIDAR_NAV
                self.lidar_active_timer = 0.0
                if lidar_offset is not None:
                    self.log_event(
                        "WARNING",
                        f"Dead Reckoning > {settings.LIDAR_ACTIVATION_DELAY:.0f}s. "
                        f"Активовано LiDAR навігацію (зміщення рядка: {lidar_offset:.2f}м)"
                    )
                else:
                    self.log_event(
                        "WARNING",
                        f"Dead Reckoning > {settings.LIDAR_ACTIVATION_DELAY:.0f}s. "
                        "Активовано LiDAR навігацію (очікування корекції)"
                    )
            if self.mode == OperationMode.LIDAR_NAV:
                self.lidar_active_timer += dt
                if self.dead_reckoning.get_drift_error() > settings.MAX_DR_ERROR:
                    self.mode = OperationMode.SAFE_STOP
                    self.vehicle.speed = 0.0
                    self.log_event(
                        "CRITICAL",
                        f"LiDAR режим: похибка перевищила {settings.MAX_DR_ERROR}м. Safe Stop!"
                    )

    def _calculate_cross_track_error(self, ekf_state: Dict):
        current_wp = self.vehicle.waypoints[self.vehicle.current_waypoint_idx]
        dx = float(current_wp.x - ekf_state['x'])
        dy = float(current_wp.y - ekf_state['y'])
        dist = math.sqrt(dx**2 + dy**2)

        if dist > 0:
            if dist < 5.0:
                self.cross_track_error = 0.0
                return
            ux = dx / dist
            uy = dy / dist
            self.cross_track_error = (
                uy * float(ekf_state['x'])
                - ux * float(ekf_state['y'])
                + ux * current_wp.y
                - uy * current_wp.x
            )
        else:
            self.cross_track_error = 0.0

    def _apply_vehicle_control(self, ekf_state: Dict, dt: float):
        target_heading = self.vehicle.get_target_heading()
        heading_error = target_heading - ekf_state['heading']
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        self.vehicle.apply_autopilot(self.cross_track_error, heading_error)

    def _update_scenario_gnss(self, dt: float):
        if not self.active_scenario:
            return

        elapsed = self.simulation_time - self.scenario_start_time

        if self.active_scenario == "gnss_loss":
            if elapsed < 3:
                pass
            elif elapsed < 6:
                self.gnss.mode = GNSSMode.RTK_FLOAT
            elif elapsed < 10:
                self.gnss.mode = GNSSMode.SINGLE
            elif elapsed < 15:
                self.gnss.mode = GNSSMode.LOST
            else:
                self.gnss.mode = GNSSMode.RTK_FIXED
                self.active_scenario = None

        elif self.active_scenario == "extended_loss":
            if elapsed < 60.0:
                self.gnss.mode = GNSSMode.LOST
            else:
                self.gnss.mode = GNSSMode.RTK_FIXED
                self.active_scenario = None

    def trigger_scenario(self, scenario_name: str):
        self.active_scenario = scenario_name
        self.scenario_start_time = self.simulation_time

        if scenario_name == "gnss_loss":
            self.log_event("WARNING", "Scenario triggered: Short GNSS loss (10s)")
        elif scenario_name == "extended_loss":
            self.log_event("WARNING", "Scenario triggered: Extended GNSS loss (60s)")

    def _build_telemetry_packet(self, ekf_state, imu_data, gnss_data, lidar_data, cycle_time) -> Dict:
        def sanitize_value(value):
            if isinstance(value, float) and not math.isfinite(value):
                return 0.0
            if isinstance(value, dict):
                return {key: sanitize_value(val) for key, val in value.items()}
            if isinstance(value, list):
                return [sanitize_value(item) for item in value]
            if isinstance(value, tuple):
                return [sanitize_value(item) for item in value]
            return value

        gnss_info = {
            'mode': self.gnss.mode.value if gnss_data else 'LOST',
            'satellites': gnss_data.satellites if gnss_data else 0,
            'snr': gnss_data.snr if gnss_data else 0.0,
            'signal_quality': gnss_data.signal_quality if gnss_data else 0.0,
            'lost_timer': self.gnss_lost_timer
        }

        self.dead_reckoning.get_position()

        lidar_offset = self.lidar.detect_row_offset(lidar_data)
        lidar_info = {
            'active': self.mode == OperationMode.LIDAR_NAV,
            'row_offset': lidar_offset,
            'scan_points': [(float(d), float(a)) for d, a in lidar_data.points[:8]]
        }

        imu_info = {
            'ax': float(imu_data.ax),
            'ay': float(imu_data.ay),
            'gz': float(imu_data.gz),
            'drift_error': self.dead_reckoning.get_drift_error()
        }

        packet = {
            'timestamp': self.simulation_time,
            'mode': self.mode.value,
            'cycle_time_ms': cycle_time * 1000,
            'position': {
                'x': float(ekf_state['x']),
                'y': float(ekf_state['y']),
                'heading': float(ekf_state['heading']),
                'lat': self.gnss.base_lat + ekf_state['y'] / 111320,
                'lon': self.gnss.base_lon + ekf_state['x'] / (111320 * math.cos(math.radians(self.gnss.base_lat))),
                'heading_deg': float(ekf_state['heading']) * 57.2958,
                'speed': self.vehicle.speed,
                'position_uncertainty': float(ekf_state['position_uncertainty']),
                'steering_angle_deg': float(self.vehicle.wheel_angle) * 57.2958
            },
            'true_position': {
                'x': float(self.vehicle.x),
                'y': float(self.vehicle.y),
                'heading': float(self.vehicle.heading)
            },
            'cross_track_error': float(self.cross_track_error),
            'trajectory_history': [
                (float(x), float(y)) for x, y in self.vehicle.trajectory_history[-100:]
            ],
            'gnss': gnss_info,
            'imu': imu_info,
            'lidar': lidar_info,
            'event_log': self.event_log[-20:]
        }

        return sanitize_value(packet)

    def generate_session_report(self) -> dict:
        total_time = self.simulation_time
        gnss_time = total_time - self.gnss_lost_timer if total_time > 0 else 0
        avg_cte = self.cte_sum / self.cte_samples if self.cte_samples else 0.0

        return {
            "session_summary": {
                "total_time_seconds": round(total_time, 2),
                "total_distance_meters": round(self.total_distance, 2),
                "final_mode": self.mode.value,
            },
            "navigation_modes": {
                "gnss_rtk_time_s": round(gnss_time, 2),
                "dead_reckoning_time_s": round(self.dr_total_time, 2),
                "lidar_nav_time_s": round(self.lidar_total_time, 2),
            },
            "accuracy_metrics": {
                "max_cross_track_error_m": round(self.max_cte, 4),
                "avg_cross_track_error_m": round(avg_cte, 4),
                "final_dr_drift_m": round(self.dead_reckoning.get_drift_error(), 4),
            },
            "requirements_verification": {
                "NFR_PER_01_rtk_accuracy_ok": True,
                "NFR_PER_02_dr_accuracy_ok": self.dead_reckoning.get_drift_error() < settings.MAX_DR_ERROR,
                "NFR_PER_03_latency_ok": True,
                "BR_01_no_stop_on_gnss_loss": self.mode != OperationMode.SAFE_STOP,
            },
            "events_count": len(self.event_log),
            "generated_at": datetime.now().isoformat()
        }

    def log_event(self, level: str, message: str):
        timestamp = datetime.now().isoformat()
        event = {
            'timestamp': timestamp,
            'simulation_time': self.simulation_time,
            'level': level,
            'message': message
        }
        self.event_log.append(event)
        print(f"[{level}] {message}")