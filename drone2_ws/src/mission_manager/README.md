# Mission Manager Node

**Fully Autonomous Drone-2 Mission Controller**

## Overview

Central state machine for Drone-2 spray missions. **Stays IDLE until first geotag**, then auto-arms, flies spray cycles, and RTLs after timeout.

## State Machine

```
IDLE (disarmed, waiting)
  │
  ▼ [first geotag received]
ARMING
  │
  ▼ [armed]
TAKING_OFF
  │
  ▼ [at altitude]
NAVIGATING ────────────────┐
  │                        │
  ▼ [arrived]              │
DETECTING                  │
  │                        │
  ▼ [confirmed]            │
CENTERING                  │
  │                        │
  ▼ [centered]             │
SPRAYING                   │
  │                        │
  ▼ [spray done]           │
WAITING_FOR_NEXT           │
  │                        │
  ├─[geotag within 5s]─────┘
  │
  ▼ [5s timeout]
RETURNING_HOME
  │
  ▼ [landed]
LANDED → IDLE
```

## Topics

### Subscribers

| Topic                            | Type              | Description         |
| -------------------------------- | ----------------- | ------------------- |
| `/drone2/target_geotag`          | `GeoPointStamped` | Target from GCS     |
| `/drone2/arrival_status`         | `Bool`            | Arrived at target   |
| `/drone2/local_detection_status` | `Bool`            | Detection confirmed |
| `/drone2/spray_ready`            | `Bool`            | Centering complete  |
| `/drone2/spray_done`             | `Bool`            | Spray complete      |
| `/mavros/state`                  | `State`           | FC armed/mode       |
| `/mavros/local_position/pose`    | `PoseStamped`     | Current altitude    |

### Publishers

| Topic                         | Type        | Description        |
| ----------------------------- | ----------- | ------------------ |
| `/drone2/status`              | `String`    | Current state name |
| `/drone2/target_position`     | `NavSatFix` | Nav target         |
| `/drone2/new_target_received` | `Bool`      | Nav trigger        |

## Parameters

| Parameter              | Default | Description                  |
| ---------------------- | ------- | ---------------------------- |
| `wait_timeout_sec`     | `5.0`   | Time to wait for next geotag |
| `takeoff_altitude_m`   | `10.0`  | Operating altitude           |
| `arming_timeout_sec`   | `30.0`  | Max time to arm              |
| `takeoff_timeout_sec`  | `60.0`  | Max time to reach altitude   |
| `altitude_tolerance_m` | `1.5`   | Altitude arrival threshold   |

## MAVROS Services

```python
/mavros/cmd/arming      # Arm on first geotag
/mavros/set_mode        # GUIDED for ops, RTL for return
/mavros/cmd/takeoff     # Initial takeoff
```

## Key Behaviors

### First Geotag Trigger

```python
if state == IDLE and not first_geotag_received:
    first_geotag_received = True
    transition(ARMING)
    request_guided_and_arm()
```

### 5-Second Wait Window

```python
def handle_waiting():
    if new_geotag_received_during_wait:
        dispatch_target(new_geotag)  # Resume cycle
    elif elapsed >= wait_timeout:
        request_rtl()  # No more targets
```

### Never Arms Without Mission

- `IDLE` state = disarmed, no geotag
- First geotag triggers full sequence
- Deterministic behavior guaranteed

## Usage

```bash
# Launch as part of sprayer system
ros2 launch drone2_bringup drone2_sprayer.launch.py

# Monitor status
ros2 topic echo /drone2/status
```

## Statistics

On shutdown, logs:

```
Final: completed=N, failed=M
```
