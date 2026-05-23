import pytest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from navigation.nav_controller import NavigationController, OperationMode
from simulation.gnss_simulator import GNSSMode
from config import settings


@pytest.fixture
def controller():
    ctrl = NavigationController()
    ctrl.initialize()
    return ctrl


class TestNavControllerInit:

    def test_initializes_in_gnss_mode(self, controller):
        assert controller.mode == OperationMode.GNSS_RTK

    def test_simulation_time_starts_at_zero(self, controller):
        assert controller.simulation_time == 0.0

    def test_vehicle_at_start_position(self, controller):
        assert controller.vehicle.x >= 0
        assert controller.vehicle.y >= 0


class TestNavControllerReset:

    def test_reset_returns_to_initial_state(self, controller):
        for _ in range(10):
            controller.step(0.1)

        time_after_steps = controller.simulation_time
        assert time_after_steps > 0

        controller.reset()

        assert controller.simulation_time == 0.0
        assert controller.mode == OperationMode.GNSS_RTK
        assert controller.gnss_lost_timer == 0.0

    def test_reset_clears_event_log(self, controller):
        for _ in range(5):
            controller.step(0.1)

        controller.reset()
        assert len(controller.event_log) <= 2


class TestNavControllerStep:

    def test_step_advances_time(self, controller):
        controller.step(0.1)
        assert abs(controller.simulation_time - 0.1) < 0.001

    def test_step_returns_telemetry_dict(self, controller):
        result = controller.step(0.1)
        assert isinstance(result, dict)

    def test_telemetry_has_required_fields(self, controller):
        result = controller.step(0.1)
        required = ['timestamp', 'mode', 'position', 'gnss', 'imu', 'event_log']
        for field in required:
            assert field in result, f"Відсутнє поле: {field}"

    def test_position_has_lat_lon(self, controller):
        result = controller.step(0.1)
        assert 'lat' in result['position']
        assert 'lon' in result['position']

    def test_lat_lon_realistic_for_ukraine(self, controller):
        result = controller.step(0.1)
        lat = result['position']['lat']
        lon = result['position']['lon']
        assert 44 < lat < 53, f"Широта {lat} не в Україні"
        assert 22 < lon < 41, f"Довгота {lon} не в Україні"

    def test_speed_realistic(self, controller):
        result = controller.step(0.1)
        speed = result['position']['speed']
        assert 0 <= speed <= 10, f"Нереалістична швидкість: {speed} м/с"


class TestNavControllerModeSwitch:
    def test_starts_in_gnss_mode(self, controller):
        result = controller.step(0.1)
        assert result['mode'] == 'GNSS_RTK'

    def test_gnss_loss_triggers_dead_reckoning(self, controller):
        controller.trigger_scenario('gnss_loss')

        for _ in range(150):
            result = controller.step(0.1)

        assert result['mode'] in ['DEAD_RECKONING', 'LIDAR_NAV', 'SAFE_STOP']

    def test_extended_loss_triggers_lidar(self, controller):
        controller.trigger_scenario('extended_loss')

        mode_history = []
        for _ in range(450):
            result = controller.step(0.1)
            mode_history.append(result['mode'])

        assert 'LIDAR_NAV' in mode_history, \
            f"LiDAR_NAV не з'явився. Режими: {set(mode_history)}"

    def test_gnss_recovery_returns_to_rtk(self, controller):
        controller.trigger_scenario('gnss_loss')

        for _ in range(100):
            controller.step(0.1)

        final_mode = None
        for _ in range(700):
            result = controller.step(0.1)
            final_mode = result['mode']

        assert final_mode == 'GNSS_RTK', \
            f"Після відновлення GNSS очікується GNSS_RTK, але: {final_mode}"

    def test_lidar_activation_does_not_drop_position_to_zero(self, controller, monkeypatch):
        controller.mode = OperationMode.DEAD_RECKONING
        controller.gnss.mode = GNSSMode.LOST
        controller.gnss_lost_timer = settings.LIDAR_ACTIVATION_DELAY
        controller.vehicle.speed = 0.0

        monkeypatch.setattr(controller.lidar, 'detect_row_offset', lambda scan: 3.0)
        monkeypatch.setattr(controller.lidar, 'try_get_position_correction', lambda scan: (0.0, 3.0))

        y_before = controller.ekf.get_state()['y']
        result = controller.step(0.1)
        y_after = controller.ekf.get_state()['y']

        assert result['mode'] == 'LIDAR_NAV'
        assert y_after > y_before - 5.0, f"LiDAR викликав різке падіння по y: {y_before:.2f} -> {y_after:.2f}"
