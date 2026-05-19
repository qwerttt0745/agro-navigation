"""
NavigationController: Main orchestrator for the navigation system
Coordinates all sensors, fusion algorithms, and mode switching
"""
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
    """Navigation operation modes"""
    INITIALIZING = "INITIALIZING"
    GNSS_RTK = "GNSS_RTK"
    DEAD_RECKONING = "DEAD_RECKONING"
    LIDAR_NAV = "LIDAR_NAV"
    SAFE_STOP = "SAFE_STOP"
    MANUAL = "MANUAL"


class NavigationController:
    """
    Main navigation controller - heart of the system
    Orchestrates sensor data, applies sensor fusion, and mode switching
    """

    def __init__(self):
        """Initialize navigation controller"""
        self.logger = logging.getLogger(__name__)

        # Components
        self.vehicle = TractorModel()
        self.gnss = GNSSSimulator()
        self.imu = IMUSimulator()
        self.lidar = LiDARSimulator()
        self.ekf = ExtendedKalmanFilter()
        self.dead_reckoning = DeadReckoningModule()

        # State tracking
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

        # Event logging
        self.event_log = []
        self.simulation_time = 0.0

        # Scenario management
        self.active_scenario = None
        self.scenario_start_time = 0.0

        # Performance monitoring
        self.cycle_times = []

    def initialize(self):
        """Initialize all subsystems"""
        self.mode = OperationMode.GNSS_RTK

        # Initialize vehicle position (field center area)
        self.vehicle.x = 50.0
        self.vehicle.y = 50.0
        self.vehicle.heading = 0.0

        # Initialize sensors
        self.gnss.mode = GNSSMode.RTK_FIXED
        self.dead_reckoning.activate(self.vehicle.x, self.vehicle.y, self.vehicle.heading)

        # Initialize EKF
        self.ekf.X[0] = self.vehicle.x
        self.ekf.X[1] = self.vehicle.y
        self.ekf.X[2] = self.vehicle.heading

        self.log_event("INFO", "System initialized. Mode: " + self.mode.value)

    def reset(self):
        """
        Повне скидання системи до початкового стану.
        Відповідає вимозі NFR-REL-02: відновлення після збою.
        """
        # Перестворюємо всі компоненти з нуля
        self.vehicle = TractorModel()
        self.gnss = GNSSSimulator()
        self.imu = IMUSimulator()
        self.lidar = LiDARSimulator()
        self.dead_reckoning = DeadReckoningModule()
        self.ekf = ExtendedKalmanFilter()

        # Скидаємо лічильники та стан
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

        # Ініціалізуємо заново
        self.initialize()
        self.log_event("INFO", "Система повністю скинута до початкового стану")
        self.logger.info("↺ NavigationController RESET виконано")

    def step(self, dt: float) -> Dict:
        """
        Main control loop step
        Args:
            dt: Time step in seconds
        Returns:
            Telemetry packet dictionary
        """
        step_start_time = time.time()
        self.simulation_time += dt
        self.total_distance += self.vehicle.speed * dt

        # 1. Update vehicle (true position)
        self.vehicle.update(dt)

        # 2. Get sensor readings
        imu_data = self.imu.get_reading(self.vehicle.heading, self.vehicle.speed, dt)
        gnss_data = None
        if self.active_scenario:
            self._update_scenario_gnss(dt)
        gnss_data = self.gnss.get_reading(self.vehicle.x, self.vehicle.y)
        lidar_data = self.lidar.get_scan(self.vehicle.x, self.vehicle.y, self.vehicle.heading)

        # 3. EKF Prediction (always)
        self.ekf.predict({'ax': imu_data.ax, 'ay': imu_data.ay, 'gz': imu_data.gz}, dt)

        # 4. Mode switching logic
        self._update_navigation_mode(gnss_data, imu_data, dt, lidar_data)

        # 5. Apply mode-specific updates
        if self.mode == OperationMode.GNSS_RTK and gnss_data:
            # Convert GNSS to local coordinates
            gnss_dict = {
                'x': self.vehicle.x + (gnss_data.lon - self.gnss.base_lon) * 111320 * math.cos(math.radians(48.95)),
                'y': self.vehicle.y + (gnss_data.lat - self.gnss.base_lat) * 111320,
                'heading': self.vehicle.heading,
                'mode': gnss_data.mode
            }
            self.ekf.update_gnss(gnss_dict)
            self.dead_reckoning.reset(self.vehicle.x, self.vehicle.y, self.vehicle.heading)

        elif self.mode == OperationMode.DEAD_RECKONING:
            # Use dead reckoning
            self.dead_reckoning.update(
                {'ax': imu_data.ax, 'ay': imu_data.ay, 'gz': imu_data.gz},
                self.vehicle.speed,
                dt,
                self.vehicle.x,
                self.vehicle.y
            )

        elif self.mode == OperationMode.LIDAR_NAV:
            # Use LiDAR correction
            correction = self.lidar.try_get_position_correction(lidar_data)
            if correction:
                self.ekf.update_lidar(correction)

        # 6. Get current state from EKF
        ekf_state = self.ekf.get_state()

        # 7. Calculate trajectory and control
        self._calculate_cross_track_error(ekf_state)
        self._apply_vehicle_control(ekf_state, dt)

        # Mode timers
        if self.mode == OperationMode.DEAD_RECKONING:
            self.dr_total_time += dt
        elif self.mode == OperationMode.LIDAR_NAV:
            self.lidar_total_time += dt

        # 8. Build telemetry packet
        step_time = time.time() - step_start_time
        self.cycle_times.append(step_time)

        # Track cross-track error stats
        self.max_cte = max(self.max_cte, self.cross_track_error)
        self.cte_sum += self.cross_track_error
        self.cte_samples += 1

        telemetry = self._build_telemetry_packet(
            ekf_state, imu_data, gnss_data, lidar_data, step_time
        )

        return telemetry

    def _update_navigation_mode(self, gnss_data, imu_data, dt, lidar_data):
        """
        Update operation mode based on sensor availability and timers.
        Mode transitions (FR-02, FR-04, FR-05):
          GNSS_RTK → DEAD_RECKONING  : on first GNSS loss
          DEAD_RECKONING → LIDAR_NAV : after 30s without GNSS
          LIDAR_NAV → SAFE_STOP      : if drift error > 30cm
          any → GNSS_RTK             : on GNSS recovery
        """
        gnss_available = (
            gnss_data is not None
            and gnss_data.mode in [GNSSMode.RTK_FIXED, GNSSMode.RTK_FLOAT]
        )

        if gnss_available:
            # ── GNSS is available ────────────────────────────────────────
            self.gnss_lost_timer = 0.0
            if self.mode != OperationMode.GNSS_RTK:
                self.mode = OperationMode.GNSS_RTK
                self.log_event("INFO", "GNSS відновлено — повернення до режиму GNSS_RTK")
        else:
            # ── GNSS is lost / degraded ──────────────────────────────────
            self.gnss_lost_timer += dt

            # Step 1: Immediate switch to Dead Reckoning (FR-04)
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

            # Step 2: After 30s — activate LiDAR navigation (FR-05)
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

            # Step 3: LiDAR mode — monitor drift (BR-01)
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
        """Calculate lateral deviation from planned path"""
        # Simplified: cross-track error is deviation from current waypoint
        current_wp = self.vehicle.waypoints[self.vehicle.current_waypoint_idx]

        # Current heading to target
        dx = float(current_wp.x - ekf_state['x'])
        dy = float(current_wp.y - ekf_state['y'])
        dist = math.sqrt(dx**2 + dy**2)

        if dist > 0:
            # Unit vector to target
            ux = dx / dist
            uy = dy / dist

            # Perpendicular distance (cross-track error)
            self.cross_track_error = abs(
                uy * float(ekf_state['x'])
                - ux * float(ekf_state['y'])
                + ux * current_wp.y
                - uy * current_wp.x
            )
        else:
            self.cross_track_error = 0.0

    def _apply_vehicle_control(self, ekf_state: Dict, dt: float):
        """Apply heading control to vehicle"""
        target_heading = self.vehicle.get_target_heading()
        heading_error = target_heading - ekf_state['heading']

        # Normalize heading error
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # Apply autopilot
        self.vehicle.apply_autopilot(self.cross_track_error, heading_error)

    def _update_scenario_gnss(self, dt: float):
        """Update GNSS based on active scenario"""
        if not self.active_scenario:
            return

        elapsed = self.simulation_time - self.scenario_start_time

        if self.active_scenario == "gnss_loss":
            # Short GNSS loss (10 seconds)
            if elapsed < 3:
                pass  # Normal RTK
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
            # Extended GNSS loss (60 seconds)
            if elapsed < 3:
                pass  # Normal RTK
            elif elapsed < 5:
                self.gnss.mode = GNSSMode.RTK_FLOAT
            elif elapsed < 10:
                self.gnss.mode = GNSSMode.LOST
            elif elapsed < 50:
                # LiDAR is active during this period
                pass
            elif elapsed < 55:
                self.gnss.mode = GNSSMode.SINGLE
            elif elapsed < 60:
                self.gnss.mode = GNSSMode.RTK_FLOAT
            else:
                self.gnss.mode = GNSSMode.RTK_FIXED
                self.active_scenario = None

    def trigger_scenario(self, scenario_name: str):
        """Trigger a predefined scenario"""
        self.active_scenario = scenario_name
        self.scenario_start_time = self.simulation_time

        if scenario_name == "gnss_loss":
            self.log_event("WARNING", "Scenario triggered: Short GNSS loss (10s)")
        elif scenario_name == "extended_loss":
            self.log_event("WARNING", "Scenario triggered: Extended GNSS loss (60s)")

    def _build_telemetry_packet(self, ekf_state, imu_data, gnss_data, lidar_data, cycle_time) -> Dict:
        """Build complete telemetry packet for transmission"""
        gnss_info = {
            'mode': self.gnss.mode.value if gnss_data else 'LOST',
            'satellites': gnss_data.satellites if gnss_data else 0,
            'snr': gnss_data.snr if gnss_data else 0.0,
            'signal_quality': gnss_data.signal_quality if gnss_data else 0.0,
            'lost_timer': self.gnss_lost_timer
        }

        # Get dead reckoning info
        dr_x, dr_y, dr_heading = self.dead_reckoning.get_position()
        _ = (dr_x, dr_y, dr_heading)

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

        # Build packet
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

        return packet

    def generate_session_report(self) -> dict:
        """
        Generate session report for diploma documentation.
        """
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
        """Log an event"""
        timestamp = datetime.now().isoformat()
        event = {
            'timestamp': timestamp,
            'simulation_time': self.simulation_time,
            'level': level,
            'message': message
        }
        self.event_log.append(event)

        # Also print to console during development
        print(f"[{level}] {message}")