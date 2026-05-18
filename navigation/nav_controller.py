"""
NavigationController: Main orchestrator for the navigation system
Coordinates all sensors, fusion algorithms, and mode switching
"""
import math
import time
from enum import Enum
from typing import Dict, List, Optional
from datetime import datetime

from simulation.vehicle import TractorModel
from simulation.gnss_simulator import GNSSSimulator, GNSSMode
from simulation.imu_simulator import IMUSimulator
from simulation.lidar_simulator import LiDARSimulator
from navigation.ekf import ExtendedKalmanFilter
from navigation.dead_reckoning import DeadReckoningModule


class OperationMode(Enum):
    """Navigation operation modes"""
    INITIALIZING = "INITIALIZING"
    GNSS_RTK = "GNSS_RTK"
    DEAD_RECKONING = "DEAD_RECKONING"
    LIDAR_NAV = "LIDAR_NAV"
    SAFE_STOP = "SAFE_STOP"
    MANUAL = "MANUAL"


class NavigationController:
    \"\"\"
    Main navigation controller - heart of the system
    Orchestrates sensor data, applies sensor fusion, and mode switching
    \"\"\"
    
    def __init__(self):
        \"\"\"Initialize navigation controller\"\"\"
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
        
        # Event logging
        self.event_log = []
        self.simulation_time = 0.0
        
        # Scenario management
        self.active_scenario = None
        self.scenario_start_time = 0.0
        
        # Performance monitoring
        self.cycle_times = []
    
    def initialize(self):
        \"\"\"Initialize all subsystems\"\"\"
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
        
        self.log_event(\"INFO\", \"System initialized. Mode: \" + self.mode.value)
    
    def reset(self):
        \"\"\"Reset simulation to initial state\"\"\"
        self.__init__()
        self.initialize()
        self.log_event(\"INFO\", \"System reset\")
    
    def step(self, dt: float) -> Dict:
        \"\"\"
        Main control loop step
        Args:
            dt: Time step in seconds
        Returns:
            Telemetry packet dictionary
        \"\"\"
        step_start_time = time.time()
        self.simulation_time += dt
        
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
        
        # 8. Build telemetry packet
        step_time = time.time() - step_start_time
        self.cycle_times.append(step_time)
        
        telemetry = self._build_telemetry_packet(
            ekf_state, imu_data, gnss_data, lidar_data, step_time
        )
        
        return telemetry
    
    def _update_navigation_mode(self, gnss_data, imu_data, dt, lidar_data):
        \"\"\"Update operation mode based on sensor availability and timers\"\"\"
        
        if gnss_data is not None and gnss_data.mode in [GNSSMode.RTK_FIXED, GNSSMode.RTK_FLOAT]:
            # GNSS available
            self.gnss_lost_timer = 0.0
            if self.mode != OperationMode.GNSS_RTK:
                self.mode = OperationMode.GNSS_RTK
                self.log_event(\"INFO\", \"GNSS RTK active - mode switched to GNSS_RTK\")
        else:
            # GNSS lost
            self.gnss_lost_timer += dt
            
            if self.gnss_lost_timer < 0.1:
                # First loss detected
                if self.mode == OperationMode.GNSS_RTK:
                    self.mode = OperationMode.DEAD_RECKONING
                    self.dead_reckoning.activate(
                        self.vehicle.x,
                        self.vehicle.y,
                        self.vehicle.heading
                    )
                    self.log_event(\"WARNING\", \"GNSS signal lost. Switching to Dead Reckoning (IMU)\")
            
            elif self.gnss_lost_timer >= 30.0:
                # 30 seconds without GNSS - activate LiDAR
                if self.mode == OperationMode.DEAD_RECKONING:
                    lidar_offset = self.lidar.detect_row_offset(lidar_data)
                    if lidar_offset is not None:
                        self.mode = OperationMode.LIDAR_NAV
                        self.lidar_active_timer = 0.0
                        self.log_event(\"WARNING\", f\"Dead Reckoning >30s. Activating LiDAR navigation (offset: {lidar_offset:.2f}m)\")
                    else:
                        # Check if drift is too large
                        if self.dead_reckoning.get_drift_error() > 2.0:
                            self.mode = OperationMode.SAFE_STOP
                            self.vehicle.speed = 0.0
                            self.log_event(\"CRITICAL\", \"Position error exceeded 2m. Safe Stop activated - manual intervention required!\")
            
            # LiDAR navigation active
            if self.mode == OperationMode.LIDAR_NAV:
                self.lidar_active_timer += dt
                
                # Check if should exit LiDAR nav
                if self.dead_reckoning.get_drift_error() > 2.0:
                    self.mode = OperationMode.SAFE_STOP
                    self.vehicle.speed = 0.0
                    self.log_event(\"CRITICAL\", \"Position error in LiDAR mode exceeded threshold. Safe Stop!\")
    
    def _calculate_cross_track_error(self, ekf_state: Dict):
        \"\"\"Calculate lateral deviation from planned path\"\"\"
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
            self.cross_track_error = abs(uy * float(ekf_state['x']) - ux * float(ekf_state['y']) + ux * current_wp.y - uy * current_wp.x)
        else:
            self.cross_track_error = 0.0
    
    def _apply_vehicle_control(self, ekf_state: Dict, dt: float):
        \"\"\"Apply heading control to vehicle\"\"\"
        target_heading = self.vehicle.get_target_heading()
        heading_error = target_heading - ekf_state['heading']
        
        # Normalize heading error
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        
        # Apply autopilot
        self.vehicle.apply_autopilot(self.cross_track_error, heading_error)
    
    def _update_scenario_gnss(self, dt: float):
        \"\"\"Update GNSS based on active scenario\"\"\"
        if not self.active_scenario:
            return
        
        elapsed = self.simulation_time - self.scenario_start_time
        
        if self.active_scenario == \"gnss_loss\":
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
        
        elif self.active_scenario == \"extended_loss\":
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
        \"\"\"Trigger a predefined scenario\"\"\"
        self.active_scenario = scenario_name
        self.scenario_start_time = self.simulation_time
        
        if scenario_name == \"gnss_loss\":
            self.log_event(\"WARNING\", \"Scenario triggered: Short GNSS loss (10s)\")
        elif scenario_name == \"extended_loss\":
            self.log_event(\"WARNING\", \"Scenario triggered: Extended GNSS loss (60s)\")
    
    def _build_telemetry_packet(self, ekf_state, imu_data, gnss_data, lidar_data, cycle_time) -> Dict:
        \"\"\"Build complete telemetry packet for transmission\"\"\"
        
        gnss_info = {
            'mode': self.gnss.mode.value if gnss_data else 'LOST',
            'satellites': gnss_data.satellites if gnss_data else 0,
            'snr': gnss_data.snr if gnss_data else 0.0,
            'signal_quality': gnss_data.signal_quality if gnss_data else 0.0,
            'lost_timer': self.gnss_lost_timer
        }
        
        # Get dead reckoning info
        dr_x, dr_y, dr_heading = self.dead_reckoning.get_position()
        
        lidar_offset = self.lidar.detect_row_offset(lidar_data)
        lidar_info = {
            'active': self.mode == OperationMode.LIDAR_NAV,
            'row_offset': lidar_offset,
            'scan_points': [(float(d), float(a)) for d, a in lidar_data.points[:8]]  # Limit transmitted points
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
                'position_uncertainty': float(ekf_state['position_uncertainty'])
            },
            'true_position': {
                'x': float(self.vehicle.x),
                'y': float(self.vehicle.y),
                'heading': float(self.vehicle.heading)
            },
            'cross_track_error': float(self.cross_track_error),
            'trajectory_history': [(float(x), float(y)) for x, y in self.vehicle.trajectory_history[-100:]],
            'gnss': gnss_info,
            'imu': imu_info,
            'lidar': lidar_info,
            'event_log': self.event_log[-20:]  # Last 20 events
        }
        
        return packet
    
    def log_event(self, level: str, message: str):
        \"\"\"Log an event\"\"\"
        timestamp = datetime.now().isoformat()
        event = {
            'timestamp': timestamp,
            'simulation_time': self.simulation_time,
            'level': level,
            'message': message
        }
        self.event_log.append(event)
        
        # Also print to console during development
        print(f\"[{level}] {message}\")
