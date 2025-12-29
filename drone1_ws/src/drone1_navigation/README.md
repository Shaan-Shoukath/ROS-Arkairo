# Drone-1 Navigation Node

**Fully Autonomous Survey Execution**

## Overview

This node executes survey missions with **zero manual intervention**. When lane segments are received from the KML planner, it automatically arms, takes off, flies the survey pattern, and returns to launch.

## State Machine

```
IDLE
  │
  ▼ [lane_segments received]
WAITING_FOR_GPS
  │
  ▼ [GPS acquired]
ARMING → TAKING_OFF
  │
  ▼ [at altitude]
NAVIGATING_TO_START → FOLLOWING_LANE → TRANSITIONING
  │                         ↑                │
  │                         └────────────────┘
  ▼ [all lanes complete]
MISSION_COMPLETE → RETURNING_HOME → LANDED
```

## Topics

### Subscribers

| Topic                            | Type               | Description                       |
| -------------------------------- | ------------------ | --------------------------------- |
| `/mission/lane_segments`         | `LaneSegmentArray` | Survey waypoints from KML planner |
| `/mavros/global_position/global` | `NavSatFix`        | Current GPS position              |
| `/mavros/local_position/pose`    | `PoseStamped`      | Current pose for altitude         |
| `/mavros/state`                  | `State`            | Armed/mode status                 |

### Publishers

| Topic                              | Type                   | Description                      |
| ---------------------------------- | ---------------------- | -------------------------------- |
| `/mavros/setpoint_position/global` | `GlobalPositionTarget` | Target position                  |
| `/drone1/next_waypoint`            | `NavSatFix`            | Current target for monitoring    |
| `/drone1/navigation_status`        | `String`               | Current state name               |
| `/drone1/detection_enable`         | `Bool`                 | Enables detection during mission |

## Parameters

| Parameter            | Default | Description                     |
| -------------------- | ------- | ------------------------------- |
| `auto_arm`           | `true`  | Auto-arm when mission received  |
| `auto_takeoff`       | `true`  | Auto-takeoff after arming       |
| `auto_rtl`           | `true`  | Auto-RTL after mission complete |
| `takeoff_altitude_m` | `6.7`   | Takeoff altitude (22 feet)      |
| `waypoint_radius_m`  | `3.0`   | Arrival detection radius        |
| `cruise_speed_mps`   | `5.0`   | Navigation speed                |
| `max_arm_retries`    | `5`     | Max arming retry attempts       |

## MAVROS Services Used

```python
/mavros/cmd/arming      # CommandBool - Arm/disarm
/mavros/set_mode        # SetMode - GUIDED/RTL
/mavros/cmd/takeoff     # CommandTOL - Takeoff
```

## Usage

```bash
# Launch with defaults (fully autonomous)
ros2 launch drone1_bringup drone1_survey.launch.py

# Manual mode (requires manual arming)
ros2 run drone1_navigation drone1_navigation_node --ros-args \
  -p auto_arm:=false -p auto_takeoff:=false
```

## Algorithms

### Haversine Distance

```python
# Used for waypoint arrival detection
a = sin(Δlat/2)² + cos(lat1)·cos(lat2)·sin(Δlon/2)²
c = 2·asin(√a)
d = R·c  # R = 6371000m
```

### Bearing Calculation

```python
# Used for yaw orientation
yaw = atan2(sin(Δlon)·cos(lat2), cos(lat1)·sin(lat2) - sin(lat1)·cos(lat2)·cos(Δlon))
```

## Safety Features

- Never arms without valid mission
- Arming timeout (30s default)
- Takeoff timeout (60s default)
- Always has RTL fallback
- All state transitions logged
