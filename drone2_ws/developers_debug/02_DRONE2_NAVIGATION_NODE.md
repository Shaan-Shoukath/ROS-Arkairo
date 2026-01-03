# Drone 2 Navigation Node - Developer Documentation

## Overview

**File**: `drone2_ws/src/drone2_navigation/drone2_navigation/drone2_navigation_node.py`  
**Package**: `drone2_navigation`  
**Node Name**: `drone2_navigation_node`  
**Author**: Shaan Shoukath

## Purpose

Receives GPS geotags from Drone-1 via telemetry and navigates to each location for spraying. Features timeout-based RTL, dynamic target acceptance, and resumable navigation.

## State Machine

```
IDLE → WAIT_FCU → WAIT_TARGET → SET_GUIDED → ARM → TAKEOFF → WAIT_TAKEOFF → NAVIGATE → ARRIVED → WAIT_FOR_NEXT → RTL → LANDED
                       ↑                                                        │
                       └────────────────────────────────────────────────────────┘
```

## Key Features

- **Dynamic Target Acceptance**: New geotags accepted in any flight state
- **RTL Resume**: If RTL triggered by timeout, can resume on new target
- **Wait Countdown**: Visible countdown during WAIT_FOR_NEXT state
- **Arrival Status**: Publishes to trigger detection node

## Key Parameters

Located in: `config/navigation_params.yaml`

```yaml
# >>> CHANGE THESE FOR DIFFERENT FLIGHT HEIGHT <<<
takeoff_altitude_m: 6.7 # 22 feet
navigation_altitude_m: 6.7 # 22 feet

arrival_radius_m: 3.0 # Consider "arrived" within this distance
wait_timeout_sec: 15.0 # Wait for next geotag before RTL
setpoint_rate_hz: 10.0 # Must be ≥10Hz
```

## Subscribers

| Topic                            | Type        | Purpose               |
| -------------------------------- | ----------- | --------------------- |
| `/drone2/target_position`        | NavSatFix   | Geotag from telem_rx  |
| `/mavros/state`                  | State       | FCU connection & mode |
| `/mavros/local_position/pose`    | PoseStamped | Current position      |
| `/mavros/global_position/global` | NavSatFix   | GPS position          |

## Publishers

| Topic                             | Type        | Purpose                 |
| --------------------------------- | ----------- | ----------------------- |
| `/mavros/setpoint_position/local` | PoseStamped | Position commands       |
| `/drone2/arrival_status`          | Bool        | Triggers detection node |
| `/drone2/navigation_status`       | String      | Current state           |

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
cd ~/Documents/ROSArkairo/drone2_ws && source install/setup.zsh
ros2 run drone2_navigation drone2_navigation_node --ros-args --params-file src/drone2_navigation/config/navigation_params.yaml
```

### T4: Send Target

```bash
ros2 topic pub /drone2/target_position sensor_msgs/msg/NavSatFix "{latitude: 10.0481, longitude: 76.3306, altitude: 10.0}" --once
```

---

**Last Updated**: January 2, 2026
