"""
GNSSSimulator: Simulates GNSS/RTK receiver with realistic signal degradation
"""
import math
import random
from enum import Enum
from dataclasses import dataclass
from typing import Optional


class GNSSMode(Enum):
    """GNSS signal mode"""
    RTK_FIXED = "RTK_FIXED"
    RTK_FLOAT = "RTK_FLOAT"
    SINGLE = "SINGLE"
    LOST = "LOST"


@dataclass
class GNSSData:
    """GNSS measurement data"""
    lat: float
    lon: float
    altitude: float
    mode: GNSSMode
    satellites: int
    snr: float  # signal-to-noise ratio in dBHz
    signal_quality: float  # 0.0 to 1.0


class GNSSSimulator:
    """
    Simulates GNSS/RTK receiver with realistic noise and signal loss scenarios
    """
    
    def __init__(self, base_lat: float = 48.9500, base_lon: float = 32.1000):
        """
        Args:
            base_lat: Base latitude of field center
            base_lon: Base longitude of field center
        """
        self.base_lat = base_lat
        self.base_lon = base_lon
        
        # Signal parameters
        self.mode = GNSSMode.RTK_FIXED
        self.signal_quality = 1.0
        self.satellites_visible = 12
        self.snr = 45.0  # dBHz
        
        # Signal loss scenario timeline (relative seconds from loss trigger)
        self.loss_scenario = {
            0: GNSSMode.RTK_FIXED,      # Start normal
            3: GNSSMode.RTK_FLOAT,       # Degrade to float
            6: GNSSMode.SINGLE,          # Further degrade
            10: GNSSMode.LOST,           # Complete loss
            40: GNSSMode.SINGLE,         # Begin recovery
            50: GNSSMode.RTK_FLOAT,
            60: GNSSMode.RTK_FIXED,
        }
        
        self.scenario_active = False
        self.scenario_start_time = 0.0
        self._scenario_name = None
        self._scenario_duration = 0.0
        self._time = 0.0
    
    def _lat_lon_to_local(self, lat: float, lon: float) -> tuple:
        """Convert lat/lon to local x, y coordinates"""
        x = (lon - self.base_lon) * 111320 * math.cos(math.radians(self.base_lat))
        y = (lat - self.base_lat) * 111320
        return x, y
    
    def _local_to_lat_lon(self, x: float, y: float) -> tuple:
        """Convert local x, y to lat/lon coordinates"""
        lat = self.base_lat + y / 111320
        lon = self.base_lon + x / (111320 * math.cos(math.radians(self.base_lat)))
        return lat, lon
    
    def _get_noise_std_dev(self) -> float:
        """Get noise standard deviation based on current mode"""
        noise_map = {
            GNSSMode.RTK_FIXED: 0.02,    # 2 cm
            GNSSMode.RTK_FLOAT: 0.15,    # 15 cm
            GNSSMode.SINGLE: 2.0,         # 2 meters
            GNSSMode.LOST: 0.0,           # No signal
        }
        return noise_map.get(self.mode, 2.0)
    
    def _get_satellite_count(self) -> int:
        """Get number of visible satellites based on mode"""
        sat_map = {
            GNSSMode.RTK_FIXED: random.randint(10, 14),
            GNSSMode.RTK_FLOAT: random.randint(7, 10),
            GNSSMode.SINGLE: random.randint(4, 7),
            GNSSMode.LOST: 0,
        }
        return sat_map.get(self.mode, 12)
    
    def _get_snr(self) -> float:
        """Get signal-to-noise ratio based on mode"""
        snr_map = {
            GNSSMode.RTK_FIXED: random.uniform(40, 50),
            GNSSMode.RTK_FLOAT: random.uniform(30, 40),
            GNSSMode.SINGLE: random.uniform(20, 30),
            GNSSMode.LOST: 0.0,
        }
        return snr_map.get(self.mode, 45.0)
    
    def get_reading(self, true_x: float, true_y: float) -> Optional[GNSSData]:
        """
        Get GNSS measurement with noise
        Args:
            true_x, true_y: True position in local coordinates
        Returns:
            GNSSData with noisy measurement, or None if signal lost
        """
        self._time += 0.1
        if self.scenario_active:
            self.update_mode_for_scenario(self._time - self.scenario_start_time)

        if self.mode == GNSSMode.LOST:
            return None
        
        # Add Gaussian noise to position
        noise_std = self._get_noise_std_dev()
        x_noisy = true_x + random.gauss(0, noise_std)
        y_noisy = true_y + random.gauss(0, noise_std)
        
        # Convert to lat/lon
        lat, lon = self._local_to_lat_lon(x_noisy, y_noisy)
        
        # Get current signal parameters
        self.satellites_visible = self._get_satellite_count()
        self.snr = self._get_snr()
        
        return GNSSData(
            lat=lat,
            lon=lon,
            altitude=120.5,  # Typical elevation
            mode=self.mode,
            satellites=self.satellites_visible,
            snr=self.snr,
            signal_quality=self.signal_quality
        )
    
    def simulate_signal_loss(self, current_time: float) -> Optional[GNSSData]:
        """
        Simulate signal loss and recovery scenario
        Call this instead of get_reading() when scenario is active
        Args:
            current_time: Current simulation time in seconds
        Returns:
            GNSSData or None based on scenario
        """
        return None  # Will be called at appropriate time
    
    def trigger_signal_loss_scenario(self, start_time: float):
        """Start the signal loss scenario"""
        self.scenario_active = True
        self.scenario_start_time = start_time
        self._scenario_name = "gnss_loss"
        self._scenario_duration = 60.0

    def trigger_scenario(self, scenario_name: str, duration: float = 60.0):
        """
        Start GNSS degradation scenario.

        Scenarios:
            'gnss_loss'     - short loss (10 s)
            'extended_loss' - long loss (60 s)
            'reb_attack'    - jamming simulation (90 s)
        """
        self._scenario_name = scenario_name
        self._scenario_start = self._time
        self._scenario_duration = duration
        self.scenario_active = True
        self.scenario_start_time = self._time

        if scenario_name == 'gnss_loss':
            self._scenario_duration = 10.0
        elif scenario_name == 'extended_loss':
            self._scenario_duration = 60.0
        elif scenario_name == 'reb_attack':
            self._scenario_duration = 90.0
    
    def update_mode_for_scenario(self, elapsed_time: float):
        """Update GNSS mode based on scenario timeline"""
        if not self.scenario_active:
            return

        if self._scenario_duration and elapsed_time >= self._scenario_duration:
            self.scenario_active = False
            self.mode = GNSSMode.RTK_FIXED
            self.signal_quality = 1.0
            return
        
        # Find the appropriate mode based on elapsed time
        mode_times = sorted(self.loss_scenario.keys())
        current_mode = GNSSMode.RTK_FIXED
        
        for time_point in mode_times:
            if elapsed_time >= time_point:
                current_mode = self.loss_scenario[time_point]
            else:
                break
        
        self.mode = current_mode
        
        # Update signal quality
        if self.mode == GNSSMode.RTK_FIXED:
            self.signal_quality = 1.0
        elif self.mode == GNSSMode.RTK_FLOAT:
            self.signal_quality = 0.7
        elif self.mode == GNSSMode.SINGLE:
            self.signal_quality = 0.3
        else:
            self.signal_quality = 0.0
    
    def is_rtk_fixed(self) -> bool:
        """Check if in RTK Fixed mode"""
        return self.mode == GNSSMode.RTK_FIXED
    
    def is_signal_degraded(self, snr_threshold: float = 35.0) -> bool:
        """Check if signal is degraded (SNR below threshold)"""
        return self.snr < snr_threshold
