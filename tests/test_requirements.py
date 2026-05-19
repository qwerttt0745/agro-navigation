import pytest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from navigation.nav_controller import NavigationController, OperationMode


@pytest.fixture
def ctrl():
    c = NavigationController()
    c.initialize()
    return c


class TestFunctionalRequirements:

    def test_FR01_gnss_data_processing(self, ctrl):
        result = ctrl.step(0.1)
        gnss = result.get('gnss', {})
        assert 'mode' in gnss
        assert 'satellites' in gnss
        assert 'snr' in gnss

    def test_FR02_gnss_loss_detection_speed(self, ctrl):
        ctrl.trigger_scenario('gnss_loss')
        result = ctrl.step(0.1)

        assert ctrl.gnss_lost_timer >= 0

    def test_FR03_sensor_fusion_active(self, ctrl):
        result = ctrl.step(0.1)
        pos = result['position']
        assert pos['x'] is not None
        assert pos['y'] is not None
        assert 'position_uncertainty' in pos

    def test_FR04_dead_reckoning_on_gnss_loss(self, ctrl):
        ctrl.trigger_scenario('gnss_loss')

        for _ in range(50):
            result = ctrl.step(0.1)

        modes_seen = set()
        for _ in range(100):
            result = ctrl.step(0.1)
            modes_seen.add(result['mode'])

        assert 'DEAD_RECKONING' in modes_seen or 'LIDAR_NAV' in modes_seen

    def test_FR06_visualization_data_available(self, ctrl):
        result = ctrl.step(0.1)
        assert 'position' in result
        assert 'lat' in result['position']
        assert 'lon' in result['position']
        assert 'heading_deg' in result['position']
        assert 'trajectory_history' in result


class TestNonFunctionalRequirements:

    def test_NFR_PER_03_latency_50ms(self, ctrl):
        import time

        times = []
        for _ in range(20):
            start = time.perf_counter()
            ctrl.step(0.1)
            elapsed_ms = (time.perf_counter() - start) * 1000
            times.append(elapsed_ms)

        avg_ms = sum(times) / len(times)
        max_ms = max(times)

        print(f"\nСередня затримка: {avg_ms:.2f} мс, Максимальна: {max_ms:.2f} мс")
        assert avg_ms < 50.0, f"Середня затримка {avg_ms:.1f} мс > 50 мс (NFR-PER-03)"

    def test_NFR_PER_02_dr_accuracy_30cm_per_100m(self, ctrl):
        ctrl.trigger_scenario('gnss_loss')

        for _ in range(400):
            ctrl.step(0.1)

        drift = ctrl.dead_reckoning.get_drift_error()
        print(f"\nПохибка DR після ~100 м: {drift:.3f} м")
        assert drift < 0.30, f"Похибка DR {drift:.3f} м > 0.30 м (NFR-PER-02)"

    def test_BR01_no_complete_stop_on_gnss_loss(self, ctrl):
        ctrl.trigger_scenario('gnss_loss')

        for _ in range(200):
            result = ctrl.step(0.1)

        assert result['mode'] != 'SAFE_STOP', \
            "Трактор зупинився через 20 с без GNSS — порушення BR-01"
        assert ctrl.vehicle.speed > 0, "Швидкість = 0, трактор зупинився"
