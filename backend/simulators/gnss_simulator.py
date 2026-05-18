"""GNSS Simulator"""
import numpy as np
from typing import Dict, Any


class GNSSSimulator:
    """Simulates GNSS receiver with signal loss scenarios"""

    def __init__(self):
        """Initialize GNSS simulator"""
        self.cycle = 0
        self.is_fixed = True
        self.x = 72000.0  # Initial position (meters)
        self.y = 13700.0
        self.z = 120.0
        self.accuracy = 0.02  # 2cm accuracy
        self.gnss_loss_start = None
        self.gnss_loss_duration = None

    def read(self) -> Dict[str, Any]:
        """Read GNSS data"""
        self.cycle += 1
        
        # Simulate GNSS signal loss at cycle 500 for 300 cycles
        if self.cycle == 500:
            self.is_fixed = False
            self.gnss_loss_start = self.cycle
            self.gnss_loss_duration = 300
        
        if (self.gnss_loss_start and 
            self.cycle > self.gnss_loss_start + self.gnss_loss_duration):
            self.is_fixed = True
        
        if not self.is_fixed:
            return {
                'is_fixed': False,
                'x': None,
                'y': None,
                'z': None,
                'snr': 0.0
            }
        
        # Add simulated movement
        self.x += 0.1 * np.sin(self.cycle * 0.01)
        self.y += 0.05 * np.cos(self.cycle * 0.01)
        self.z += 0.01 * np.sin(self.cycle * 0.02)
        
        return {
            'is_fixed': True,
            'x': self.x + np.random.normal(0, self.accuracy),
            'y': self.y + np.random.normal(0, self.accuracy),
            'z': self.z + np.random.normal(0, 0.1),
            'snr': 45.0,  # Signal-to-noise ratio (dBHz)
            'num_satellites': 12
        }

    def inject_signal_loss(self, start_cycle: int, duration: int):
        """Inject GNSS signal loss event"""
        self.gnss_loss_start = start_cycle
        self.gnss_loss_duration = duration
