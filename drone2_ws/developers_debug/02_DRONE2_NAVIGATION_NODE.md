# Drone 2 Navigation Node - Developer Documentation

## Overview

| Property        | Value                                                                         |
| --------------- | ----------------------------------------------------------------------------- |
| **File**        | `drone2_ws/src/drone2_navigation/drone2_navigation/drone2_navigation_node.py` |
| **Package**     | `drone2_navigation`                                                           |
| **Node Name**   | `drone2_navigation_node`                                                      |
| **Config File** | `drone2_navigation/config/navigation_params.yaml`                             |
| **Maintainer**  | Shaan Shoukath                                                                |

## Purpose

Receives GPS geotags from Drone-1 via telemetry and navigates to each location for spraying. Features timeout-based RTL, dynamic target acceptance, and can resume from LANDED state.

---

## State Variables

| Variable                  | Type        | Purpose                                              |
| ------------------------- | ----------- | ---------------------------------------------------- |
| `self.state`              | FlightState | Current state machine state                          |
| `self.target_gps`         | NavSatFix   | Target GPS coordinates from telem_rx                 |
| `self.target_local`       | (x, y, z)   | Target in local ENU coordinates                      |
| `self.home_gps`           | NavSatFix   | Home GPS (first fix or config)                       |
| `self.home_local`         | PoseStamped | Home in local coordinates                            |
| `self.local_pose`         | PoseStamped | Current drone position                               |
| `self.relative_altitude`  | float       | Barometer-based altitude (for takeoff detection)     |
| `self.fcu_connected`      | bool        | FCU connection state                                 |
| `self.is_armed`           | bool        | Whether drone is armed                               |
| `self.current_mode`       | str         | Current flight mode (STABILIZE, GUIDED, etc)         |
| `self.state_start_time`   | Time        | When current state was entered (for timeouts)        |
| `self.rtl_due_to_timeout` | bool        | True if RTL was triggered by timeout (allows resume) |
| `self.targets_completed`  | int         | Count of completed spray targets                     |

---

## Configuration YAML → Code Mapping

```yaml
# navigation_params.yaml                 # Python code usage
takeoff_altitude_m: 6.7           →  self.takeoff_alt (used in handle_takeoff)
navigation_altitude_m: 6.7        →  self.navigation_alt (setpoint Z height)
arrival_radius_m: 3.0             →  self.arrival_radius (distance check in handle_navigate)
wait_timeout_sec: 30.0            →  self.wait_timeout (RTL trigger in handle_wait_for_next)
setpoint_rate_hz: 10.0            →  Timer rate for publish_setpoint
use_gps_home: true                →  If false, uses start_latitude/longitude as home
start_latitude: 10.0478           →  self.config_home_lat
start_longitude: 76.3303          →  self.config_home_lon
```

---

## Key Functions and Why They Exist

### Callbacks (React to external events)

| Function                | Why It Exists                                              | Variables Used                              |
| ----------------------- | ---------------------------------------------------------- | ------------------------------------------- |
| `target_callback`       | Receives geotags from telem_rx, triggers state transitions | `target_gps`, `target_local`, `state`       |
| `state_callback`        | Monitors FCU connection/mode changes from MAVROS           | `fcu_connected`, `is_armed`, `current_mode` |
| `gps_callback`          | Captures home GPS position on first fix                    | `home_gps`, `config_home_lat/lon`           |
| `pose_callback`         | Updates current position for navigation                    | `local_pose`, `home_local`                  |
| `spray_done_callback`   | Detects when spray cycle completes                         | `state` → transitions to WAIT_FOR_NEXT      |
| `relative_alt_callback` | Barometer altitude for accurate takeoff detection          | `relative_altitude`                         |

### State Handlers (Execute state-specific logic)

| Function               | Why It Exists                                 | Variables Used                                 |
| ---------------------- | --------------------------------------------- | ---------------------------------------------- |
| `handle_set_guided`    | Request GUIDED mode, set RTL params           | `current_mode`, ArduPilot params               |
| `handle_arm`           | Stream setpoints (required!), then arm        | `is_armed`, setpoint streaming                 |
| `handle_takeoff`       | Send MAVLink takeoff command                  | `takeoff_alt`                                  |
| `handle_wait_takeoff`  | Monitor altitude climb until threshold        | `relative_altitude`, `takeoff_alt`             |
| `handle_navigate`      | Publish setpoints, check arrival distance     | `target_local`, `local_pose`, `arrival_radius` |
| `handle_arrived`       | Signal centering node, move to spray wait     | `targets_completed`, arrival_pub               |
| `handle_wait_spray`    | Wait for spray_done signal (timeout fallback) | `state_start_time`                             |
| `handle_wait_for_next` | Wait for next geotag, RTL on timeout          | `wait_timeout`, `rtl_due_to_timeout`           |
| `handle_rtl`           | Request RTL mode, monitor landing             | `current_mode`, `relative_altitude`            |

### Utility Functions

| Function                | Why It Exists                                             | Variables Used                     |
| ----------------------- | --------------------------------------------------------- | ---------------------------------- |
| `fsm_update`            | Main state machine dispatcher (2Hz)                       | `state`, calls current handler     |
| `publish_setpoint`      | Continuous position commands (10Hz required by ArduPilot) | `target_local`, `navigation_alt`   |
| `gps_to_local`          | Convert GPS to local ENU (needed for setpoints)           | `home_gps`, Earth radius constants |
| `transition_to`         | State change + logging + reset timer                      | `state`, `state_start_time`        |
| `time_in_state`         | How long in current state (for timeouts)                  | `state_start_time`                 |
| `configure_sitl_params` | Disable auto-disarm for SITL testing                      | MAVROS param service               |

---

## State Machine

```
IDLE → WAIT_FCU → WAIT_TARGET → SET_GUIDED → ARM → TAKEOFF → WAIT_TAKEOFF
    → NAVIGATE → ARRIVED → WAIT_SPRAY → WAIT_FOR_NEXT → (new target / RTL)
    → RTL → LANDED → (new geotag restarts from SET_GUIDED)
```

---

## Subscribers

| Topic                             | Type        | Callback                | Purpose              |
| --------------------------------- | ----------- | ----------------------- | -------------------- |
| `/drone2/target_position`         | NavSatFix   | `target_callback`       | Geotag from telem_rx |
| `/drone2/spray_done`              | Bool        | `spray_done_callback`   | Spray complete       |
| `/mavros/state`                   | State       | `state_callback`        | FCU status           |
| `/mavros/local_position/pose`     | PoseStamped | `pose_callback`         | Position             |
| `/mavros/global_position/global`  | NavSatFix   | `gps_callback`          | GPS                  |
| `/mavros/global_position/rel_alt` | Float64     | `relative_alt_callback` | Altitude             |

## Publishers

| Topic                             | Type        | Purpose                  |
| --------------------------------- | ----------- | ------------------------ |
| `/mavros/setpoint_position/local` | PoseStamped | Position commands (10Hz) |
| `/drone2/arrival_status`          | Bool        | Triggers centering node  |
| `/drone2/navigation_status`       | String      | Monitoring               |

---

## Launch Command

```bash
ros2 run drone2_navigation drone2_navigation_node --ros-args \
  --params-file ~/Documents/ROS-Arkairo/drone2_ws/src/drone2_navigation/config/navigation_params.yaml
```

---

**Last Updated**: January 10, 2026
