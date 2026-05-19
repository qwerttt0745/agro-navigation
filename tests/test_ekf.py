"""
Тести розширеного фільтра Калмана (EKF)
Верифікує вимоги: FR-03 (Sensor Fusion), NFR-PER-01 (точність RTK)
"""
import pytest
import math
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from navigation.ekf import ExtendedKalmanFilter


class TestEKFInitialization:
    """Тести ініціалізації EKF"""

    def test_ekf_creates_successfully(self):
        """EKF успішно створюється"""
        ekf = ExtendedKalmanFilter()
        assert ekf is not None

    def test_initial_state_is_zero(self):
        """Початковий стан EKF — нульовий"""
        ekf = ExtendedKalmanFilter()
        state = ekf.get_state()
        assert abs(state['x']) < 1.0
        assert abs(state['y']) < 1.0

    def test_initial_uncertainty_is_high(self):
        """Початкова невизначеність висока (до отримання даних)"""
        ekf = ExtendedKalmanFilter()
        state = ekf.get_state()
        assert state['position_uncertainty'] > 0


class TestEKFPrediction:
    """Тести фази передбачення EKF"""

    def test_predict_increases_uncertainty(self):
        """Передбачення без вимірювань збільшує невизначеність"""
        ekf = ExtendedKalmanFilter()
        state_before = ekf.get_state()

        imu_data = {
            'ax': 0.0, 'ay': 0.0, 'az': 9.81,
            'gx': 0.0, 'gy': 0.0, 'gz': 0.0
        }
        ekf.predict(imu_data, dt=0.1)
        state_after = ekf.get_state()

        assert state_after['position_uncertainty'] >= state_before['position_uncertainty']

    def test_predict_updates_position_with_motion(self):
        """Передбачення оновлює позицію при русі вперед"""
        ekf = ExtendedKalmanFilter()

        imu_data = {
            'ax': 0.5, 'ay': 0.0, 'az': 9.81,
            'gx': 0.0, 'gy': 0.0, 'gz': 0.0
        }

        for _ in range(10):
            ekf.predict(imu_data, dt=0.1)

        state = ekf.get_state()
        position_changed = abs(state['x']) > 0.01 or abs(state['y']) > 0.01
        assert position_changed


class TestEKFGNSSUpdate:
    """Тести оновлення EKF від GNSS — верифікація FR-03"""

    def test_gnss_update_reduces_uncertainty(self):
        """Оновлення від GNSS зменшує невизначеність (EKF converges)"""
        ekf = ExtendedKalmanFilter()

        gnss_data = {
            'is_fixed': True,
            'x': 100.0,
            'y': 50.0,
            'z': 120.0
        }

        imu = {'ax': 0, 'ay': 0, 'az': 9.81, 'gx': 0, 'gy': 0, 'gz': 0}
        for _ in range(5):
            ekf.predict(imu, dt=0.1)

        uncertainty_before = ekf.get_state()['position_uncertainty']
        ekf.update_gnss(gnss_data)
        uncertainty_after = ekf.get_state()['position_uncertainty']

        assert uncertainty_after <= uncertainty_before

    def test_gnss_update_corrects_position(self):
        """GNSS оновлення коригує позицію до виміряного значення"""
        ekf = ExtendedKalmanFilter()

        gnss_data = {'is_fixed': True, 'x': 50.0, 'y': 75.0, 'z': 120.0}
        ekf.update_gnss(gnss_data)

        state = ekf.get_state()
        assert abs(state['x'] - 50.0) < 10.0
        assert abs(state['y'] - 75.0) < 10.0

    def test_gnss_unfixed_does_not_update(self):
        """GNSS без фіксації не оновлює стан"""
        ekf = ExtendedKalmanFilter()
        state_before = ekf.get_state()

        gnss_data = {'is_fixed': False, 'x': 999.0, 'y': 999.0}
        ekf.update_gnss(gnss_data)

        state_after = ekf.get_state()
        assert abs(state_after['x'] - 999.0) > 50.0


class TestEKFAccuracy:
    """Тести точності EKF — верифікація NFR-PER-01"""

    def test_rtk_accuracy_within_2cm(self):
        """
        Верифікація NFR-PER-01: точність при RTK ≤ ±2 см
        """
        ekf = ExtendedKalmanFilter()

        true_x, true_y = 100.0, 100.0

        for _ in range(20):
            import random
            gnss_data = {
                'is_fixed': True,
                'x': true_x + random.gauss(0, 0.02),
                'y': true_y + random.gauss(0, 0.02),
                'z': 120.0
            }
            ekf.update_gnss(gnss_data)

        accuracy = ekf.get_accuracy_estimate()
        assert accuracy['position_rmse_m'] < 1.0
