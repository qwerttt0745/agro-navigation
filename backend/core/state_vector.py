"""Navigation system state vector and data structures"""
from dataclasses import dataclass, asdict
from datetime import datetime
from enum import Enum
from typing import Dict, Any
import json


class SourceEnum(Enum):
    """Navigation source enumeration"""
    GNSS = "GNSS"
    DEAD_RECKONING = "DEAD_RECKONING"
    LIDAR = "LIDAR"
    FUSION = "FUSION"


@dataclass
class StateVector:
    """Navigation system state vector (9D)"""
    x: float  # Position X (meters)
    y: float  # Position Y (meters)
    z: float  # Height (meters)
    heading: float  # Heading (radians, 0 = North)
    velocity_x: float  # Velocity X (m/s)
    velocity_y: float  # Velocity Y (m/s)
    roll: float  # Roll angle (radians)
    pitch: float  # Pitch angle (radians)
    yaw_rate: float  # Yaw rate (rad/s)
    position_uncertainty: float  # Position error (meters)
    source: SourceEnum  # Navigation source
    timestamp: datetime = None

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now()

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary"""
        data = asdict(self)
        data['source'] = self.source.value
        data['timestamp'] = self.timestamp.isoformat()
        data['heading_deg'] = round(float(self.heading * 57.2958), 2)  # Convert to degrees
        data['velocity_ms'] = round(float((self.velocity_x**2 + self.velocity_y**2)**0.5), 3)
        return data

    def to_json(self) -> str:
        """Convert to JSON string"""
        return json.dumps(self.to_dict(), indent=2)

    def is_position_reliable(self, threshold: float = 0.3) -> bool:
        """Check if position is reliable"""
        return self.position_uncertainty <= threshold
