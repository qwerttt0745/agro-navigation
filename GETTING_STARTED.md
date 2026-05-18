# 🚜 Agro Navigation System - Getting Started Guide

## Overview

This is a fully functional **navigation system for agricultural machinery** featuring:
- **Extended Kalman Filter** for sensor fusion (GNSS, IMU, LiDAR)
- **Dead Reckoning** fallback when GNSS is lost
- **Interactive Web Dashboard** with real-time visualization
- **10 Hz Navigation Cycles** with steering commands
- **Automatic Mode Switching** (4 operating modes)

## System Requirements

- **Python 3.9+**
- **Modern Web Browser** (Chrome, Firefox, Safari, Edge)
- **Operating System:** Linux, macOS, or Windows

## ⚡ Quick Start (5 minutes)

### Step 1: Navigate to Project
```bash
cd /home/oleksii/Desktop/Agro-Navigation/agro_nav
```

### Step 2: Activate Virtual Environment
```bash
# The virtual environment is already set up
# Just verify with:
python --version  # Should show Python 3.9.25
```

### Step 3: Start the Server
```bash
python -m uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

Expected output:
```
INFO:     Uvicorn running on http://0.0.0.0:8000
INFO:     Application startup complete
```

### Step 4: Open Dashboard
Open your browser and go to:
```
http://localhost:8000
```

You should see the **Agro Navigation System Dashboard** with:
- 🗺️ Map on the left
- 📊 Status panel and controls on the right
- Charts for tracking navigation data

### Step 5: Start Simulation
1. Click the **▶ Start** button
2. Watch the map show vehicle movement
3. Observe real-time charts updating
4. See GNSS_FIXED mode in operation

**Congratulations! The system is running! 🎉**

---

## 🎬 What to Observe

### Real-Time Dashboard Updates

The dashboard updates **10 times per second** showing:

**Status Panel:**
- **Position X/Y:** Vehicle coordinates in meters
- **Heading:** Direction in degrees (0° = North)
- **Speed:** Current velocity in m/s
- **Steering Angle:** Angle commands to vehicle
- **Position Error:** Estimated uncertainty in position
- **Cross-Track Error:** Deviation from planned path

**Charts:**
1. **Position History (XY)** - Trajectory plot
2. **Error Evolution** - Position uncertainty growing over time
3. **Heading & Speed** - Two overlaid time series

**Map:**
- Blue marker shows vehicle position
- Blue line shows the trajectory trail
- Real-time pan/zoom as vehicle moves

**Event Log:**
- Mode transitions
- System events
- Timestamped entries

### Simulated GNSS Signal Loss

At **cycle 500 (50 seconds):**
- GNSS signal is lost
- System automatically switches to **DEAD_RECKONING**
- Mode indicator turns orange
- Position uncertainty starts growing
- Steering continues based on IMU data

After **cycle 800:**
- GNSS signal returns
- System switches back to **GNSS_FIXED**
- Mode indicator turns green
- Position uncertainty stabilizes

---

## 🧪 Running Tests

Test the system's core components:

```bash
# Run all 12 tests
pytest tests/ -v

# Run specific test file
pytest tests/test_navigation_system.py -v

# Run with detailed output
pytest tests/ -vv --tb=long
```

Expected output:
```
============================= 12 passed in 0.19s ==============================

✓ test_dead_reckoning_activation
✓ test_dead_reckoning_integration
✓ test_dead_reckoning_error_accumulation
✓ test_dead_reckoning_error_threshold
✓ test_navigation_system_initialization
✓ test_navigation_cycle
✓ test_multiple_cycles
✓ test_work_report
✓ test_ekf_initialization
✓ test_ekf_predict
✓ test_ekf_gnss_update
✓ test_ekf_covariance_reduction
```

---

## 🔧 Common Commands

### Stop the Server
```
Ctrl + C  (in the terminal where server is running)
```

### Reset Dashboard
Click the **🔄 Reset** button to:
- Clear all history
- Reset position to initial
- Restart navigation system

### Pause Simulation
Click the **⏹ Stop** button to:
- Pause all navigation cycles
- Freeze current state
- Pause data streaming

### Check Server Status
```bash
# In a new terminal
curl http://localhost:8000/status

# This returns JSON with:
# - running: true/false
# - connected_clients: number
# - status: final report
```

---

## 📊 Understanding the Output

### Navigation Cycle Output

Each cycle (100 ms) produces:

```json
{
  "cycle": 150,                          # Cycle number
  "mode": "GNSS_FIXED",                 # Operation mode
  "timestamp": "2026-05-18T21:30:45.123",
  "state": {
    "x": 72045.32,                      # Position X (meters)
    "y": 13702.15,                      # Position Y (meters)
    "z": 120.5,                         # Height (meters)
    "heading": 0.25,                    # Heading (radians, 0=North)
    "velocity_x": 0.5,                  # X velocity (m/s)
    "velocity_y": 5.2,                  # Y velocity (m/s)
    "roll": 0.0,                        # Roll angle
    "pitch": 0.0,                       # Pitch angle
    "yaw_rate": 0.01,                   # Yaw rate (rad/s)
    "position_uncertainty": 0.02,       # Error (meters)
    "source": "GNSS",                   # Data source
    "heading_deg": 14.3,                # Heading in degrees
    "velocity_ms": 5.25                 # Speed in m/s
  },
  "cross_track_error": 0.15,            # Deviation from path
  "steering_angle_deg": 2.5,            # Steering command
  "speed_mps": 5.0                      # Target speed
}
```

### Mode Meanings

| Mode | When | Color | Duration |
|------|------|-------|----------|
| GNSS_FIXED | GNSS available | 🟢 Green | 0-30s |
| DEAD_RECKONING | GNSS lost | 🟠 Orange | 30-120s |
| LIDAR_NAV | After 30s loss | 🟣 Purple | 30-120s |
| SAFE_STOP | After 120s loss | 🔴 Red | >120s |

---

## 🔍 Exploring the Code

### Key Files to Understand

1. **`backend/core/state_vector.py`**
   - Defines the 9D navigation state
   - Handles state-to-JSON conversion

2. **`backend/core/sensor_fusion.py`**
   - Extended Kalman Filter implementation
   - Sensor fusion logic

3. **`backend/navigation/navigation_system.py`**
   - Main orchestrator
   - Mode switching logic
   - Event generation

4. **`frontend/index.html`**
   - Dashboard UI
   - Real-time charting
   - WebSocket connection

### Example: Manual Navigation Cycle

```python
from agro_nav.backend.navigation.navigation_system import NavigationSystem

# Create and initialize
nav_system = NavigationSystem()
nav_system.initialize()

# Run one cycle
result = nav_system.run_navigation_cycle()

# Access results
print(f"Mode: {result['mode']}")
print(f"Position: ({result['state']['x']:.2f}, {result['state']['y']:.2f})")
print(f"Error: {result['state']['position_uncertainty']:.3f} m")

# Run 100 cycles (10 seconds)
for i in range(100):
    result = nav_system.run_navigation_cycle()
    if i % 10 == 0:
        print(f"Cycle {result['cycle']}: {result['mode']}")

# Get final report
report = nav_system.generate_work_report()
print(report)
```

---

## ❓ Troubleshooting

### Server won't start
```bash
# Check if port 8000 is in use
lsof -i :8000

# Kill the process if needed
kill -9 <PID>

# Try different port
python -m uvicorn main:app --port 8001
```

### Dashboard doesn't load
- Check browser console (F12)
- Verify server is running
- Try different browser
- Clear browser cache

### WebSocket errors
- Check browser console for errors
- Verify you clicked "Start" button
- Look for error messages in server terminal
- Check network tab in DevTools

### Tests fail
```bash
# Check Python version
python --version  # Should be 3.9+

# Reinstall dependencies
pip install -r requirements.txt --force-reinstall

# Run again
pytest tests/ -v
```

### Import errors
```bash
# Ensure you're in the right directory
cd agro_nav

# Reinstall package in development mode
pip install -e .

# Try importing manually
python -c "from backend.navigation.navigation_system import NavigationSystem"
```

---

## 📚 Further Reading

- **`README.md`** - Full project documentation
- **`PROJECT_SUMMARY.md`** - Comprehensive technical overview
- **Code docstrings** - Detailed API documentation
- **Test files** - Usage examples
- **`frontend/index.html`** - Dashboard implementation details

---

## 🎯 Learning Path

1. **Understand the System**
   - Read PROJECT_SUMMARY.md
   - Look at backend/core/state_vector.py
   - Review operation modes in navigation_system.py

2. **Run It Live**
   - Start the server
   - Open the dashboard
   - Watch mode switching when GNSS is lost

3. **Test Components**
   - Run pytest tests
   - Read test code for usage examples
   - Modify tests to experiment

4. **Modify and Extend**
   - Change mode switching thresholds in .env
   - Modify sensor simulator parameters
   - Add new navigation features

---

## 📞 Support

For issues:
1. Check the Troubleshooting section above
2. Review test files for usage examples
3. Check server terminal for error messages
4. Inspect browser console (F12) for frontend issues
5. Read code comments and docstrings

---

## ✅ Verification Checklist

- [ ] Python 3.9+ installed
- [ ] Virtual environment activated
- [ ] Dependencies installed (`pip install -r requirements.txt`)
- [ ] All 12 tests pass (`pytest tests/ -v`)
- [ ] Server starts without errors
- [ ] Dashboard loads in browser
- [ ] Start button responds and data streams
- [ ] Mode switching occurs at cycle 500
- [ ] Charts update in real-time
- [ ] Event log shows transitions

---

## 🎉 You're All Set!

The Agro Navigation System is ready to use. Start the server, open the dashboard, and explore how a real navigation system handles GNSS signal loss with automatic fallback to Dead Reckoning and LiDAR navigation.

**Happy navigating! 🚜**

---

*For complete documentation, see PROJECT_SUMMARY.md*
*Project Status: ✅ Production Ready*
