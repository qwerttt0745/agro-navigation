# Agro Navigation System

Advanced GNSS/IMU/LiDAR sensor fusion navigation system for agricultural machinery with real-time fault tolerance and mode switching.

## Features

- **Extended Kalman Filter (EKF)** for 9-dimensional state estimation
- **GNSS Signal Loss Detection** with automatic fallback to Dead Reckoning
- **Dead Reckoning Module** for autonomous positioning via IMU integration
- **LiDAR SLAM** for terrain-based navigation in GNSS-denied areas
- **Real-time Mode Switching**: GNSS_FIXED → DEAD_RECKONING → LIDAR_NAV → SAFE_STOP
- **Interactive WebSocket-based Dashboard** with live position tracking
- **Comprehensive Sensor Simulators** for GNSS, IMU, LiDAR, and vehicle dynamics
- **Full Test Suite** with pytest

## Quick Start

### 1. Install Dependencies

```bash
cd Agro-Navigation
pip install -r requirements.txt
```

### 2. Run the Server

```bash
python -m uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

### 3. Open Dashboard

Visit http://localhost:8000 in your browser

### 4. Run Tests

```bash
pytest
```

## Operation Modes

| Mode | Duration | Description |
|------|----------|-------------|
| GNSS_FIXED | 0-30s | Primary navigation with full accuracy |
| DEAD_RECKONING | 30-120s | IMU-based positioning |
| LIDAR_NAV | 30-120s | Terrain-based SLAM navigation |
| SAFE_STOP | >120s | Emergency stop mode |

## Performance Specifications

- Navigation Cycle: 10 Hz (100 ms)
- End-to-end Latency: < 50 ms
- GNSS Accuracy: ±2 cm (RTK)
- Dead Reckoning Accuracy: ≤ 30 cm per 100 m
