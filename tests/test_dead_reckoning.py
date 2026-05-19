"""
Tests for Dead Reckoning module
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from navigation.dead_reckoning import DeadReckoningModule


def test_drift_error_grows_with_distance():
    dr = DeadReckoningModule()
    dr.activate(0.0, 0.0, 0.0)

    # Simulate 100 m at 2.5 m/s
    for _ in range(40 * 10):
        dr.update({'gz': 0.0}, wheel_speed=2.5, dt=0.1)

    drift = dr.get_drift_error()
    assert 0.0 < drift < 0.30
