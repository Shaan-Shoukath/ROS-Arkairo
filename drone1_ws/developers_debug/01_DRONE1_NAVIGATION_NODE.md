# Drone 1 Navigation Node - Developer Documentation

## Overview

**File**: `drone1_ws/src/drone1_navigation/drone1_navigation/drone1_navigation_node.py`  
**Package**: `drone1_navigation`  
**Node Name**: `drone1_navigation_node`  
**Author**: Shaan Shoukath

## Purpose

Autonomous waypoint navigation for Drone-1 (survey drone). Receives lane segments from KML planner and executes sequential waypoint following with disease detection enabled during flight.

## State Machine

```
IDLE → WAIT_FCU → SET_GUIDED → ARM → TAKEOFF → WAIT_TAKEOFF → NAVIGATE → LANE_COMPLETE → RTL
```

## Key Parameters

Located in: `config/navigation_params.yaml`

```yaml
# >>> CHANGE THIS FOR DIFFERENT FLIGHT HEIGHT <<<
mission_altitude: 6.7 # 22 feet

setpoint_rate_hz: 10.0 # Must be ≥10Hz for ArduPilot
waypoint_radius_m: 3.0 # Arrival tolerance
fcu_timeout_sec: 30.0
arming_timeout_sec: 60.0
takeoff_timeout_sec: 90.0
```

## Subscribers

| Topic                            | Type        | Purpose                    |
| -------------------------------- | ----------- | -------------------------- |
| `/drone1/lane_segment`           | LaneSegment | Waypoints from KML planner |
| `/mavros/state`                  | State       | FCU connection & mode      |
| `/mavros/local_position/pose`    | PoseStamped | Current position           |
| `/mavros/global_position/global` | NavSatFix   | GPS position               |

## Publishers

| Topic                             | Type        | Purpose                  |
| --------------------------------- | ----------- | ------------------------ |
| `/mavros/setpoint_position/local` | PoseStamped | Position commands        |
| `/drone1/detection_enable`        | Bool        | Enable/disable detection |
| `/drone1/navigation_status`       | String      | Current state            |

## SITL Testing

### T1: SITL

```bash
cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter --console --map -l 10.0478,76.3303,0,0 -w
```

### T2: MAVROS

```bash
ros2 launch mavros apm.launch.py fcu_url:=udp://:14550@127.0.0.1:14555
```

### T3: Navigation Node

```bash
cd ~/Documents/ROSArkairo/drone1_ws && source install/setup.zsh
ros2 run drone1_navigation drone1_navigation_node --ros-args --params-file src/drone1_navigation/config/navigation_params.yaml
```

### T4: KML Planner (provides waypoints)

```bash
cd ~/Documents/ROSArkairo/drone1_ws && source install/setup.zsh
ros2 run kml_lane_planner kml_lane_planner_node
```

---

**Last Updated**: January 2, 2026
