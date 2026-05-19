import math
import random
from enum import Enum
from dataclasses import dataclass
from typing import Optional

from config import settings


class GNSSMode(Enum):
    RTK_FIXED = "RTK_FIXED"
    RTK_FLOAT = "RTK_FLOAT"
    SINGLE = "SINGLE"
    LOST = "LOST"


@dataclass
class GNSSData:
    lat: float
    lon: float
    altitude: float
    mode: GNSSMode
    satellites: int
    snr: float
    signal_quality: float


class GNSSSimulator:
    def __init__(self, base_lat: float = settings.BASE_LAT, base_lon: float = settings.BASE_LON):
        self.base_lat = base_lat
        self.base_lon = base_lon

        self.mode = GNSSMode.RTK_FIXED
        self.signal_quality = 1.0
        self.satellites_visible = 12
        self.snr = 45.0

        self.loss_scenario = {
            0: GNSSMode.RTK_FIXED,
            3: GNSSMode.RTK_FLOAT,
            6: GNSSMode.SINGLE,
            10: GNSSMode.LOST,
            40: GNSSMode.SINGLE,
            50: GNSSMode.RTK_FLOAT,
            60: GNSSMode.RTK_FIXED,
        }

        self.scenario_active = False
        self.scenario_start_time = 0.0
        self._scenario_name = None
        self._scenario_duration = 0.0
        self._time = 0.0
    
    def _lat_lon_to_local(self, lat: float, lon: float) -> tuple:
        x = (lon - self.base_lon) * 111320 * math.cos(math.radians(self.base_lat))
        y = (lat - self.base_lat) * 111320
        return x, y
    
    def _local_to_lat_lon(self, x: float, y: float) -> tuple:
        lat = self.base_lat + y / 111320
        lon = self.base_lon + x / (111320 * math.cos(math.radians(self.base_lat)))
        return lat, lon
    
    def _get_noise_std_dev(self) -> float:
        noise_map = {
            GNSSMode.RTK_FIXED: 0.02,
            GNSSMode.RTK_FLOAT: 0.15,
            GNSSMode.SINGLE: 2.0,
            GNSSMode.LOST: 0.0,
        }
        return noise_map.get(self.mode, 2.0)
    
    def _get_satellite_count(self) -> int:
        sat_map = {
            GNSSMode.RTK_FIXED: random.randint(10, 14),
            GNSSMode.RTK_FLOAT: random.randint(7, 10),
            GNSSMode.SINGLE: random.randint(4, 7),
            GNSSMode.LOST: 0,
        }
        return sat_map.get(self.mode, 12)
    
    def _get_snr(self) -> float:
        snr_map = {
            GNSSMode.RTK_FIXED: random.uniform(40, 50),
            GNSSMode.RTK_FLOAT: random.uniform(30, 40),
            GNSSMode.SINGLE: random.uniform(20, 30),
            GNSSMode.LOST: 0.0,
        }
        return snr_map.get(self.mode, 45.0)
    
    def get_reading(self, true_x: float, true_y: float) -> Optional[GNSSData]:
        self._time += 0.1
        if self.scenario_active:
            self.update_mode_for_scenario(self._time - self.scenario_start_time)

        if self.mode == GNSSMode.LOST:
            return None

        noise_std = self._get_noise_std_dev()
        x_noisy = true_x + random.gauss(0, noise_std)
        y_noisy = true_y + random.gauss(0, noise_std)

        lat, lon = self._local_to_lat_lon(x_noisy, y_noisy)

        self.satellites_visible = self._get_satellite_count()
        self.snr = self._get_snr()

        return GNSSData(
            lat=lat,
            lon=lon,
            altitude=120.5,
            mode=self.mode,
            satellites=self.satellites_visible,
            snr=self.snr,
            signal_quality=self.signal_quality
        )
    
    def simulate_signal_loss(self, current_time: float) -> Optional[GNSSData]:
        return None
    
    def trigger_signal_loss_scenario(self, start_time: float):
        self.scenario_active = True
        self.scenario_start_time = start_time
        self._scenario_name = "gnss_loss"
        self._scenario_duration = 60.0

    def trigger_scenario(self, scenario_name: str, duration: float = 60.0):
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
        if not self.scenario_active:
            return

        if self._scenario_duration and elapsed_time >= self._scenario_duration:
            self.scenario_active = False
            self.mode = GNSSMode.RTK_FIXED
            self.signal_quality = 1.0
            return

        mode_times = sorted(self.loss_scenario.keys())
        current_mode = GNSSMode.RTK_FIXED

        for time_point in mode_times:
            if elapsed_time >= time_point:
                current_mode = self.loss_scenario[time_point]
            else:
                break
        
        self.mode = current_mode
        
        if self.mode == GNSSMode.RTK_FIXED:
            self.signal_quality = 1.0
        elif self.mode == GNSSMode.RTK_FLOAT:
            self.signal_quality = 0.7
        elif self.mode == GNSSMode.SINGLE:
            self.signal_quality = 0.3
        else:
            self.signal_quality = 0.0
    
    def is_rtk_fixed(self) -> bool:
        return self.mode == GNSSMode.RTK_FIXED
    
    def is_signal_degraded(self, snr_threshold: float = 35.0) -> bool:
        return self.snr < snr_threshold
