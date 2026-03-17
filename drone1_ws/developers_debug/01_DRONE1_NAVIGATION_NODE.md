# Drone 1 Navigation Node - Developer Documentation

## Overview

| Property        | Value                                                                         |
| --------------- | ----------------------------------------------------------------------------- |
| **File**        | `drone1_ws/src/drone1_navigation/drone1_navigation/drone1_navigation_node.py` |
| **Package**     | `drone1_navigation`                                                           |
| **Node Name**   | `drone1_navigation_node`                                                      |
| **Config File** | `drone1_navigation/config/navigation_params.yaml`                             |
| **Maintainer**  | Shaan Shoukath                                                                |

## Purpose

Autonomous waypoint navigation for Drone-1 (survey drone). Flies KML-defined survey patterns at 22 feet altitude with disease detection enabled during navigation.

---

## State Variables

| Variable                 | Type          | Purpose                               |
| ------------------------ | ------------- | ------------------------------------- |
| `self.state`             | FlightState   | Current state machine state           |
| `self.waypoints`         | List[(x,y,z)] | Current lane's waypoints in local ENU |
| `self.current_wp_index`  | int           | Index of current target waypoint      |
| `self.home_gps`          | NavSatFix     | Home GPS position                     |
| `self.home_local`        | PoseStamped   | Home in local coordinates             |
| `self.local_pose`        | PoseStamped   | Current drone position                |
| `self.relative_altitude` | float         | Barometer altitude                    |
| `self.fcu_connected`     | bool          | FCU connection state                  |
| `self.is_armed`          | bool          | Armed state                           |
| `self.current_mode`      | str           | Flight mode                           |
| `self.detection_enabled` | bool          | Whether detection node is active      |
| `self.state_start_time`  | Time          | For timeout calculations              |

---

## Configuration YAML → Code Mapping

```yaml
# navigation_params.yaml                  # Where Used
mission_altitude: 6.7             →  self.mission_alt (takeoff + setpoint height)
setpoint_rate_hz: 10.0            →  Timer rate for publish_setpoint
waypoint_radius_m: 3.0            →  Distance check in handle_navigate
fcu_timeout_sec: 30.0             →  Timeout in WAIT_FCU state
arming_timeout_sec: 60.0          →  Timeout in ARM state
takeoff_timeout_sec: 90.0         →  Timeout in WAIT_TAKEOFF state
use_gps_home: true                →  If false, uses start_latitude/start_longitude
start_latitude: 10.0478           →  self.config_home_lat
start_longitude: 76.3303          →  self.config_home_lon
```

---

## Key Functions and Why They Exist

### Callbacks (React to external events)

| Function                | Why It Exists                       | Variables Used                              |
| ----------------------- | ----------------------------------- | ------------------------------------------- |
| `lane_segment_callback` | Receives waypoints from KML planner | `waypoints`, `current_wp_index`             |
| `state_callback`        | Monitors FCU connection/mode        | `fcu_connected`, `is_armed`, `current_mode` |
| `gps_callback`          | Captures home GPS on first fix      | `home_gps`                                  |
| `pose_callback`         | Updates current position            | `local_pose`, `home_local`                  |
| `relative_alt_callback` | Barometer altitude for takeoff      | `relative_altitude`                         |

### State Handlers

| Function               | Why It Exists                                     | Variables Used                                       |
| ---------------------- | ------------------------------------------------- | ---------------------------------------------------- |
| `handle_set_guided`    | Request GUIDED mode, configure RTL params         | `current_mode`, RTL_ALT param                        |
| `handle_arm`           | Stream setpoints then arm (required sequence)     | `is_armed`, setpoint streaming                       |
| `handle_takeoff`       | Send MAVLink takeoff command                      | `mission_alt`                                        |
| `handle_wait_takeoff`  | Wait until 80% altitude reached                   | `relative_altitude`, `mission_alt`                   |
| `handle_navigate`      | Fly waypoints, enable detection at first waypoint | `waypoints`, `current_wp_index`, `detection_enabled` |
| `handle_lane_complete` | Wait for next lane from KML planner               | `waypoints` cleared                                  |
| `handle_rtl`           | Request RTL, disable detection                    | `current_mode`, `detection_enabled`                  |

### Utility Functions

| Function                | Why It Exists                                        | Variables Used              |
| ----------------------- | ---------------------------------------------------- | --------------------------- |
| `fsm_update`            | Main state machine dispatcher (2Hz)                  | `state`, calls handlers     |
| `publish_setpoint`      | Continuous position commands (10Hz required)         | `waypoints`, `mission_alt`  |
| `gps_to_local`          | Convert GPS → ENU for setpoints                      | `home_gps`                  |
| `transition_to`         | State change + logging                               | `state`, `state_start_time` |
| `configure_sitl_params` | Set RTL_ALT (maintain altitude), disable auto-disarm | MAVROS param service        |

---

## Detection Enable Logic

Detection is enabled ONLY during `NAVIGATE` state:

```python
# In handle_navigate() when entering first time:
if not self.detection_enabled:
    self.detection_pub.publish(Bool(data=True))
    self.detection_enabled = True

# In handle_rtl() / handle_lane_complete():
self.detection_pub.publish(Bool(data=False))
self.detection_enabled = False
```

This ensures the detection node only processes images while actively surveying.

---

## Hardware Setup

```
┌─────────────────────────────────────────────────────────────┐
│                       DRONE 1                               │
│                                                             │
│   ┌─────────────────┐                                       │
│   │  Raspberry Pi 5 │                                       │
│   │  (Companion)    │                                       │
│   │                 │                                       │
│   │  Nodes:         │                                       │
│   │  • navigation   │                                       │
│   │  • kml_planner  │                                       │
│   │  • image_cap    │                                       │
│   │  • detection    │                                       │
│   │  • telem_tx     │                                       │
│   └────────┬────────┘                                       │
│            │ USB Cable                                      │
│            ▼                                                │
│   ┌─────────────────┐                                       │
│   │  Cube Orange+   │                                       │
│   │  SYSID = 1      │                                       │
│   │                 │                                       │
│   │  SERIAL0 (USB)  │ ← Pi (MAVLink2, 115200)               │
│   │  SERIAL1/TELEM1 │ → Radio (MAVLink2, 57600)             │
│   └────────┬────────┘                                       │
│            ▼                                                │
│   ┌─────────────────┐                                       │
│   │ Telemetry Radio │ ~~~ RF (Net ID: 25) ~~~ GCS/Drone2    │
│   └─────────────────┘                                       │
└─────────────────────────────────────────────────────────────┘
```

### ArduPilot Parameters

```
SYSID_THISMAV = 1        # Drone 1 system ID
SERIAL0_PROTOCOL = 2     # USB = MAVLink2
SERIAL0_BAUD = 115       # 115200
SERIAL0_OPTIONS = 0      # Forwarding enabled
SERIAL1_PROTOCOL = 2     # TELEM1 = MAVLink2
SERIAL1_BAUD = 57        # 57600
SERIAL1_OPTIONS = 0      # Forwarding enabled
```

---

## State Machine

```
IDLE → WAIT_FCU → SET_GUIDED → ARM → TAKEOFF → WAIT_TAKEOFF → NAVIGATE
                                                      ↑           ↓
                                                      └── (next lane_segment)
                                                                  ↓
                                    LANE_COMPLETE → (wait) → RTL → LANDED
```

---

## Subscribers

| Topic                             | Type        | Callback                | Purpose    |
| --------------------------------- | ----------- | ----------------------- | ---------- |
| `/drone1/lane_segment`            | LaneSegment | `lane_segment_callback` | Waypoints  |
| `/mavros/state`                   | State       | `state_callback`        | FCU status |
| `/mavros/local_position/pose`     | PoseStamped | `pose_callback`         | Position   |
| `/mavros/global_position/global`  | NavSatFix   | `gps_callback`          | GPS        |
| `/mavros/global_position/rel_alt` | Float64     | `relative_alt_callback` | Altitude   |

## Publishers

| Topic                             | Type        | Purpose                  |
| --------------------------------- | ----------- | ------------------------ |
| `/mavros/setpoint_position/local` | PoseStamped | Position commands (10Hz) |
| `/drone1/detection_enable`        | Bool        | Control detection node   |
| `/drone1/navigation_status`       | String      | Monitoring               |

---

## Manual Testing

### Start MAVROS (Hardware - USB)

```bash
ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200
```

### Start MAVROS (SITL)

```bash
ros2 launch mavros apm.launch fcu_url:=udp://:14551@127.0.0.1:14550
```

### Run Navigation Node

```bash
cd ~/Documents/ROS-Arkairo/drone1_ws && source install/setup.zsh
ros2 run drone1_navigation drone1_navigation_node --ros-args \
  --params-file ~/Documents/ROS-Arkairo/drone1_ws/src/drone1_navigation/config/navigation_params.yaml
```

### Monitor Status

```bash
ros2 topic echo /drone1/navigation_status
```

---

## Config File

```yaml
# navigation_params.yaml
/**:
  ros__parameters:
    mission_altitude: 6.7 # 22 feet
    setpoint_rate_hz: 10.0
    waypoint_radius_m: 3.0
    fcu_timeout_sec: 30.0
    arming_timeout_sec: 60.0
    takeoff_timeout_sec: 90.0
    use_gps_home: true
    start_latitude: 28.4215
    start_longitude: 77.5243
```

---

**Last Updated**: January 13, 2026
