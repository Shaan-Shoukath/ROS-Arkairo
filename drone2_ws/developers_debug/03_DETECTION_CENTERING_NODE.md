# Detection & Centering Node - Developer Documentation

## Overview

| Property        | Value                                                                                                   |
| --------------- | ------------------------------------------------------------------------------------------------------- |
| **File**        | `drone2_ws/src/local_detection_and_centering/local_detection_and_centering/detection_centering_node.py` |
| **Package**     | `local_detection_and_centering`                                                                         |
| **Node Name**   | `detection_centering_node`                                                                              |
| **Config File** | `local_detection_and_centering/config/detection_centering_params.yaml`                                  |
| **Maintainer**  | Shaan Shoukath                                                                                          |

## Purpose

Complete spray cycle after arrival at geotag: detect disease, center over target using visual servoing, descend, spray, ascend.

---

## State Variables

| Variable                      | Type         | Purpose                                            |
| ----------------------------- | ------------ | -------------------------------------------------- |
| `self.state`                  | State enum   | Current state (IDLE, DETECTING, CENTERING, etc)    |
| `self.current_altitude`       | float        | Current altitude from pose                         |
| `self.current_bbox`           | (x, y, w, h) | Bounding box of detected target                    |
| `self.pid`                    | PIDState     | Integral/derivative terms for PID controller       |
| `self.state_start_time`       | Time         | When current state was entered                     |
| `self.detection_attempts`     | int          | Number of frames processed for detection           |
| `self.consecutive_detections` | int          | Confirmed detections in a row                      |
| `self.use_sim`                | bool         | If true, skip detection and go straight to descent |

---

## Configuration YAML → Code Mapping

```yaml
# detection_centering_params.yaml         # Where Used
use_sim: true                     →  If true, skip DETECTING/CENTERING, go to DESCENDING
navigation_altitude_m: 6.7        →  Target altitude after ascending (handle_ascent)
spray_altitude_m: 2.44            →  Target altitude for spraying (handle_descent)
descent_velocity_mps: 0.3         →  Z velocity during descent
ascend_velocity_mps: 0.5          →  Z velocity during ascent
spray_duration_sec: 3.0           →  How long to stay in SPRAYING state
centered_threshold_pixels: 30    →  Error threshold to consider "centered"
centering_timeout_sec: 30.0       →  Max time in CENTERING before giving up
detection_timeout_sec: 30.0       →  Max time in DETECTING before giving up
kp: 0.002                         →  PID proportional gain (pixel error → velocity)
ki: 0.0001                        →  PID integral gain (accumulated error)
kd: 0.001                         →  PID derivative gain (error rate)
```

---

## Key Functions and Why They Exist

### Callbacks (React to external events)

| Function           | Why It Exists                                               | Variables Used                                 |
| ------------------ | ----------------------------------------------------------- | ---------------------------------------------- |
| `arrival_callback` | Navigation signals "arrived at geotag" - starts spray cycle | `state` → DETECTING (or DESCENDING if use_sim) |
| `image_callback`   | Receives camera frames for detection/tracking               | `current_bbox`, detection counters             |
| `pose_callback`    | Updates altitude for descent/ascent logic                   | `current_altitude`                             |

### State Handlers (Called by control_loop based on state)

| Function             | Why It Exists                                  | Variables Used                                 |
| -------------------- | ---------------------------------------------- | ---------------------------------------------- |
| `_process_detection` | Find disease in frame, build confidence        | `detection_attempts`, `consecutive_detections` |
| `_update_target`     | Track target during centering (update bbox)    | `current_bbox`                                 |
| `_handle_centering`  | PID visual servoing to align drone over target | `pid`, `current_bbox`, velocity commands       |
| `_handle_descent`    | Command downward velocity until spray altitude | `current_altitude`, `spray_altitude`           |
| `_handle_spraying`   | Wait for spray_duration, signal sprayer        | `spray_duration`, `state_start_time`           |
| `_handle_ascent`     | Command upward velocity until nav altitude     | `current_altitude`, `navigation_altitude`      |

### Utility Functions

| Function             | Why It Exists                                      | Variables Used                        |
| -------------------- | -------------------------------------------------- | ------------------------------------- |
| `control_loop`       | Main state machine (20Hz) - dispatches to handlers | `state`, calls handlers               |
| `_detect_disease`    | HSV color detection for yellow spots               | Returns bbox or None                  |
| `_publish_velocity`  | Send velocity commands to MAVROS                   | vx, vy, vz in body frame              |
| `_stop_motion`       | Zero all velocities                                | Publishes (0,0,0)                     |
| `_publish_ready`     | Signal sprayer node to activate                    | spray_ready=True                      |
| `_send_statustext`   | Send status to Mission Planner for monitoring      | MAVLink STATUSTEXT                    |
| `_transition_to`     | State change + reset timers + logging              | `state`, `state_start_time`, counters |
| `_get_state_elapsed` | How long in current state (for timeouts)           | `state_start_time`                    |

---

## State Machine

```
IDLE → DETECTING → CENTERING → DESCENDING → SPRAYING → ASCENDING → COMPLETED
  ↑        ↓            ↓           ↓                                    │
  └── (timeout) ───────┴────────────┘                                    │
  └──────────────────────────────────────────────────────────────────────┘
```

When `use_sim=true`:

```
IDLE → (arrival) → DESCENDING → SPRAYING → ASCENDING → COMPLETED → IDLE
```

---

## Visual Servoing (Centering)

```
  Image Center (cx, cy)
         │
         ▼
    ┌─────────┐
    │  Target │  ← Detected bounding box center
    └─────────┘
         │
    error_x = target_x - image_center_x
    error_y = target_y - image_center_y
         │
         ▼
    velocity = Kp*error + Ki*integral + Kd*derivative
         │
         ▼
    Publish velocity command → Drone moves → Repeat
```

Centered when: `abs(error_x) < 30 AND abs(error_y) < 30`

---

## Subscribers

| Topic                         | Type        | Callback           | Purpose          |
| ----------------------------- | ----------- | ------------------ | ---------------- |
| `/drone2/arrival_status`      | Bool        | `arrival_callback` | Trigger from nav |
| `/camera/image_raw`           | Image       | `image_callback`   | Camera frames    |
| `/mavros/local_position/pose` | PoseStamped | `pose_callback`    | Altitude         |

## Publishers

| Topic                                         | Type       | Purpose            |
| --------------------------------------------- | ---------- | ------------------ |
| `/mavros/setpoint_velocity/cmd_vel_unstamped` | Twist      | Velocity commands  |
| `/drone2/spray_ready`                         | Bool       | Sprayer activation |
| `/drone2/centering_status`                    | Bool       | Detection status   |
| `/mavros/statustext/send`                     | StatusText | Mission Planner    |

---

## Launch Command

```bash
ros2 run local_detection_and_centering detection_centering_node --ros-args \
  --params-file ~/Documents/ROS-Arkairo/drone2_ws/src/local_detection_and_centering/config/detection_centering_params.yaml

# SITL mode (skip detection/centering)
ros2 run local_detection_and_centering detection_centering_node --ros-args -p use_sim:=true
```

---

**Last Updated**: January 10, 2026
