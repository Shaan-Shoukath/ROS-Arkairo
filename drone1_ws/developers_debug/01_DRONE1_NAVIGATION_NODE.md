# Drone 1 Navigation Node - Developer Documentation

## Overview

**File**: `drone1_ws/src/drone1_navigation/drone1_navigation/drone1_navigation_node.py`  
**Package**: `drone1_navigation`  
**Node Name**: `drone1_navigation_node`  
**Author**: Shaan Shoukath

## Purpose

Autonomous waypoint navigation for Drone-1 (survey drone). Flies KML-defined survey patterns at 22 feet altitude with disease detection enabled.

---

## Hardware Preconditions

```
Raspberry Pi 5 ─── UART/USB ───► Cube Orange+
Pi Camera 3 ─── CSI ───► Raspberry Pi 5
GPS ───► Cube Orange+
```

## ArduPilot Parameters

```
SERIAL2_PROTOCOL = 2     # TELEM2 = MAVLink2 (Pi)
SERIAL2_BAUD = 921       # 921600
SYSID_THISMAV = 1        # Drone 1 system ID
ARMING_CHECK = 1         # Enable all arming checks
```

---

## State Machine

```
IDLE → WAIT_FCU → SET_GUIDED → ARM → TAKEOFF → WAIT_TAKEOFF → NAVIGATE → LANE_COMPLETE → RTL
```

## Key Parameters

```yaml
mission_altitude: 6.7 # 22 feet
setpoint_rate_hz: 10.0 # ≥10Hz required
waypoint_radius_m: 3.0 # Arrival tolerance
fcu_timeout_sec: 30.0
arming_timeout_sec: 60.0
takeoff_timeout_sec: 90.0
```

---

## Subscribers

| Topic                         | Type        | Purpose               |
| ----------------------------- | ----------- | --------------------- |
| `/drone1/lane_segment`        | LaneSegment | Waypoints from KML    |
| `/mavros/state`               | State       | FCU connection & mode |
| `/mavros/local_position/pose` | PoseStamped | Current position      |

## Publishers

| Topic                             | Type        | Purpose                  |
| --------------------------------- | ----------- | ------------------------ |
| `/mavros/setpoint_position/local` | PoseStamped | Position commands        |
| `/drone1/detection_enable`        | Bool        | Enable disease detection |
| `/drone1/navigation_status`       | String      | Current state            |

---

## Launch Command

```bash
ros2 run drone1_navigation drone1_navigation_node --ros-args \
  --params-file src/drone1_navigation/config/navigation_params.yaml
```

---

**Last Updated**: January 5, 2026
