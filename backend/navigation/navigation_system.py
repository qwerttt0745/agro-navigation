"""Main Navigation System orchestrator"""
from enum import Enum
from datetime import datetime
from typing import Dict, Any, List
import logging
from ..core.state_vector import StateVector, SourceEnum
from ..core.sensor_fusion import SensorFusionUnit
from .dead_reckoning import DeadReckoningModule
from .trajectory_planner import TrajectoryPlanner
from ..simulators.gnss_simulator import GNSSSimulator
from ..simulators.imu_simulator import IMUSimulator
from ..simulators.lidar_simulator import LiDARSimulator
from ..simulators.vehicle_simulator import VehicleSimulator


class OperationMode(Enum):
    """Navigation system operation modes"""
    INITIALIZING = "INITIALIZING"
    GNSS_FIXED = "GNSS_FIXED"
    DEAD_RECKONING = "DEAD_RECKONING"
    LIDAR_NAV = "LIDAR_NAV"
    SAFE_STOP = "SAFE_STOP"
    MANUAL = "MANUAL"


class NavigationSystem:
    """Main Navigation System"""

    def __init__(self):
        """Initialize Navigation System"""
        self.logger = logging.getLogger(__name__)
        
        # Operation mode
        self.mode = OperationMode.INITIALIZING
        
        # Core navigation
        self.state = StateVector(x=0.0, y=0.0, z=120.0, heading=0.0,
                               velocity_x=0.0, velocity_y=0.0,
                               roll=0.0, pitch=0.0, yaw_rate=0.0,
                               position_uncertainty=100.0,
                               source=SourceEnum.GNSS)
        
        # Sensor fusion
        self.ekf = SensorFusionUnit()
        
        # Dead reckoning
        self.dead_reckoning = DeadReckoningModule()
        
        # Trajectory planner
        self.trajectory_planner = TrajectoryPlanner()
        
        # Simulators
        self.gnss_sim = GNSSSimulator()
        self.imu_sim = IMUSimulator()
        self.lidar_sim = LiDARSimulator()
        self.vehicle_sim = VehicleSimulator()
        
        # Timing
        self.cycle_count = 0
        self.time_in_dr = 0.0  # Time in dead reckoning
        self.time_in_lidar = 0.0  # Time in LiDAR navigation
        self.gnss_loss_time = 0.0  # Time since GNSS loss
        
        # Events
        self.events: List[Dict[str, Any]] = []
        
        # Thresholds
        self.DR_ACTIVATION_TIME = 30.0  # seconds
        self.LIDAR_ACTIVATION_TIME = 30.0  # seconds
        self.SAFE_STOP_TIME = 120.0  # seconds
        self.ERROR_THRESHOLD = 0.3  # meters (30 cm per 100 m)
        self.SNR_THRESHOLD = 35.0  # dBHz
        
        self.dt = 0.1  # 100 ms cycle

    def initialize(self):
        """Initialize navigation system"""
        self.logger.info("Initializing Navigation System")
        self.mode = OperationMode.GNSS_FIXED
        self.state.source = SourceEnum.GNSS
        self.cycle_count = 0

    def run_navigation_cycle(self) -> Dict[str, Any]:
        """Execute one navigation cycle (100 ms)"""
        self.cycle_count += 1
        
        # Get sensor data
        gnss_data = self.gnss_sim.read()
        imu_data = self.imu_sim.read()
        lidar_data = self.lidar_sim.read()
        
        # EKF prediction with IMU
        self.state = self.ekf.predict(imu_data)
        
        # Process GNSS data
        if gnss_data.get('is_fixed', False):
            self.gnss_loss_time = 0.0
            self.ekf.update_gnss(gnss_data)
            self.mode = OperationMode.GNSS_FIXED
        else:
            self.gnss_loss_time += self.dt
        
        # Mode switching logic
        if self.gnss_loss_time > 0:
            if self.gnss_loss_time <= self.DR_ACTIVATION_TIME:
                if self.mode != OperationMode.DEAD_RECKONING:
                    self.dead_reckoning.activate()
                    self.mode = OperationMode.DEAD_RECKONING
                    self._log_event("DR_ACTIVATED", "Dead Reckoning activated")
                
                self.time_in_dr += self.dt
            
            elif self.gnss_loss_time <= self.SAFE_STOP_TIME:
                if self.mode != OperationMode.LIDAR_NAV:
                    self.mode = OperationMode.LIDAR_NAV
                    self._log_event("LIDAR_NAV_ACTIVATED", "LiDAR Navigation activated")
                
                self.time_in_lidar += self.dt
                
                # Process LiDAR data
                if lidar_data:
                    self.ekf.update_lidar(lidar_data)
            
            elif self.gnss_loss_time > self.SAFE_STOP_TIME:
                if self.dead_reckoning.get_accumulated_error() > self.ERROR_THRESHOLD:
                    self.mode = OperationMode.SAFE_STOP
                    self._log_event("SAFE_STOP_ACTIVATED", "Safe stop activated - error threshold exceeded")
        
        # Update dead reckoning if active
        if self.dead_reckoning.active:
            self.dead_reckoning.integrate_imu(
                imu_data['accel_x'], imu_data['accel_y'], imu_data['accel_z'],
                imu_data['gyro_x'], imu_data['gyro_y'], imu_data['gyro_z'],
                self.dt
            )
        
        # Generate steering command
        steering_cmd = self.trajectory_planner.generate_steering_command(
            self.state.x, self.state.y, self.state.heading, self.dt
        )
        
        # Update vehicle simulator
        self.vehicle_sim.step(steering_cmd.angle_deg, steering_cmd.speed_mps, self.dt)
        
        # Update state
        self.state = self.ekf._state_to_vector(self.state.source)
        
        # Calculate cross-track error
        cte = self.trajectory_planner.calculate_cross_track_error(
            self.state.x, self.state.y, self.state.heading
        )
        
        # Prepare output
        result = {
            'cycle': self.cycle_count,
            'mode': self.mode.value,
            'timestamp': datetime.now().isoformat(),
            'state': self.state.to_dict(),
            'cross_track_error': round(float(cte), 3),
            'steering_angle_deg': round(steering_cmd.angle_deg, 2),
            'speed_mps': round(steering_cmd.speed_mps, 2),
            'gnss_loss_time': round(self.gnss_loss_time, 2),
            'dead_reckoning_error': round(self.dead_reckoning.get_accumulated_error(), 3),
            'time_in_dr': round(self.time_in_dr, 2),
            'time_in_lidar': round(self.time_in_lidar, 2),
            'events': self.events[-5:] if self.events else []  # Last 5 events
        }
        
        self.events = []  # Clear events for next cycle
        
        return result

    def _log_event(self, event_type: str, message: str):
        """Log event"""
        self.events.append({
            'timestamp': datetime.now().isoformat(),
            'type': event_type,
            'message': message
        })
        self.logger.info(f"[{event_type}] {message}")

    def generate_work_report(self) -> Dict[str, Any]:
        """Generate work report"""
        return {
            'total_cycles': self.cycle_count,
            'current_mode': self.mode.value,
            'total_time_dr': round(self.time_in_dr, 2),
            'total_time_lidar': round(self.time_in_lidar, 2),
            'final_position': {
                'x': round(self.state.x, 2),
                'y': round(self.state.y, 2),
                'z': round(self.state.z, 2)
            },
            'final_heading_deg': round(self.state.heading * 57.2958, 2),
            'position_uncertainty': round(self.state.position_uncertainty, 3)
        }
