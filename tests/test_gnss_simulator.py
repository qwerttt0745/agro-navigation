import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from simulation.gnss_simulator import GNSSSimulator, GNSSMode
from config import settings


def test_gnss_signal_loss_scenario_changes_mode():
    gnss = GNSSSimulator(settings.BASE_LAT, settings.BASE_LON)
    gnss.trigger_scenario('gnss_loss')
    modes = []
    for _ in range(200):
        data = gnss.get_reading(0.0, 0.0)
        modes.append(gnss.mode)

    assert GNSSMode.LOST in modes or GNSSMode.SINGLE in modes
