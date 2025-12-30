# Drone2 Navigation Node - Developer Documentation

## Overview

**File**: `drone2_ws/src/drone2_navigation/drone2_navigation/drone2_navigation_node.py`  
**Package**: `drone2_navigation`  
**Node Name**: `drone2_navigation_node`  
**Author**: Shaan Shoukath  
**Purpose**: Unified autonomous flight controller for Drone-2 (sprayer missions)

## What This Node Does

Complete autonomous flight controller:

1. **Waits for target from telem_rx** (via `/drone2/target_position`)
2. **Arms motors automatically** on first target
3. **Executes takeoff** to configured altitude
4. **Navigates to GPS target** using position control
5. **Publishes arrival status** when reached
6. **Hovers and waits for next target** (configurable timeout with countdown)
7. **RTL if no new target** received (resumable if new geotag arrives during RTL)

## Key Features

### Dynamic Target Acceptance

New targets can be received in **any flight state**:

- `WAIT_TARGET` → Start flight sequence
- `NAVIGATE` → Redirect to new target immediately
- `ARRIVED` → Skip wait, navigate to new target
- `WAIT_FOR_NEXT` → Navigate to new target
- `RTL` (timeout only) → Resume navigation

### Resumable RTL

- **Timeout RTL**: If no geotag within wait period → RTL but can resume if new geotag
- **Emergency RTL**: FCU disconnect or lost GUIDED mode → stays in RTL (safety)

## State Machine

```
IDLE → WAIT_FCU → WAIT_TARGET → SET_GUIDED → ARM → TAKEOFF →
WAIT_TAKEOFF → NAVIGATE → ARRIVED → WAIT_FOR_NEXT → [NAVIGATE or RTL]
                              ↑___________________________|
                    (new geotag during RTL resumes here)
```

## Core Logic

### Target Reception (handles all states)

```python
def target_callback(self, msg: NavSatFix):
    self.target_gps = msg
    self.target_local = gps_to_local(msg)

    if state == WAIT_TARGET:
        transition_to(SET_GUIDED)  # First target → start flight
    elif state == WAIT_FOR_NEXT:
        transition_to(NAVIGATE)    # New target → continue mission
    elif state in [NAVIGATE, ARRIVED]:
        # Just update target - drone redirects automatically
        if state == ARRIVED:
            transition_to(NAVIGATE)
    elif state == RTL and rtl_due_to_timeout:
        transition_to(NAVIGATE)    # Resume from timeout RTL
```

### RTL Resume Logic

```python
def handle_navigate(self):
    # If not in GUIDED mode (e.g. resuming from RTL), request it
    if fcu_state.mode != 'GUIDED':
        request_guided_mode()
        return  # Wait for GUIDED

    # Normal navigation...
```

### Wait Period with Countdown

```python
def handle_wait_for_next(self):
    time_remaining = wait_timeout - time_in_state()

    # Log countdown every 2 seconds
    log(f'Hovering at waypoint... {time_remaining}s remaining before RTL')

    if time_in_state() >= wait_timeout:
        rtl_due_to_timeout = True  # Mark as resumable
        transition_to(RTL)
```

## Subscribers

| Topic                                  | Type        | Source   | Purpose                        |
| -------------------------------------- | ----------- | -------- | ------------------------------ |
| `/drone2/target_position`              | NavSatFix   | telem_rx | GPS targets from Drone-1       |
| `/mavros/state`                        | State       | MAVROS   | FCU connection/mode            |
| `/mavros/global_position/global`       | NavSatFix   | MAVROS   | Current GPS, home capture      |
| `/mavros/local_position/pose`          | PoseStamped | MAVROS   | Local position for navigation  |
| `/mavros/global_position/relative_alt` | Float64     | MAVROS   | Barometer altitude for takeoff |

## Publishers

| Topic                             | Type        | Rate  | Purpose                        |
| --------------------------------- | ----------- | ----- | ------------------------------ |
| `/mavros/setpoint_position/local` | PoseStamped | 10Hz  | Position commands to ArduPilot |
| `/drone2/arrival_status`          | Bool        | Event | Triggers detection/spraying    |
| `/drone2/navigation_status`       | String      | 1Hz   | Current state monitoring       |

## Parameters

All parameters configurable via `config/navigation_params.yaml`:

| Parameter               | Default | Description                                   |
| ----------------------- | ------- | --------------------------------------------- |
| `takeoff_altitude_m`    | 10.0    | Initial takeoff height                        |
| `navigation_altitude_m` | 10.0    | Cruise altitude to targets                    |
| `arrival_radius_m`      | 3.0     | Distance to consider arrived                  |
| `setpoint_rate_hz`      | 10.0    | Position command rate (≥10Hz required)        |
| `wait_timeout_sec`      | 10.0    | Time to wait for next geotag (with countdown) |
| `takeoff_timeout_sec`   | 60.0    | Max takeoff time before RTL                   |
| `fcu_timeout_sec`       | 30.0    | FCU connection timeout                        |

## Service Clients

- `/mavros/cmd/arming` - CommandBool
- `/mavros/set_mode` - SetMode (GUIDED/RTL)
- `/mavros/cmd/takeoff` - CommandTOL
- `/mavros/param/set` - ParamSetV2 (SITL config)

## SITL Testing

### Quick Test Commands

```bash
# Terminal 1: SITL (wipe state for fresh start)
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map -l 10.0478,76.3303,0,0 -w

# Terminal 2: MAVROS
ros2 launch mavros apm.launch.py fcu_url:=udp://:14550@127.0.0.1:14555

# Terminal 3: Navigation Node
cd ~/Documents/ROSArkairo/drone2_ws && source install/setup.zsh
ros2 run drone2_navigation drone2_navigation_node --ros-args \
  --params-file src/drone2_navigation/config/navigation_params.yaml

# Terminal 4: Send test targets
ros2 topic pub /drone2/target_position sensor_msgs/msg/NavSatFix \
  "{latitude: 10.0481, longitude: 76.3306, altitude: 10.0}" --once
```

### Test Locations (from home 10.0478, 76.3303)

| Target  | Latitude | Longitude | Distance |
| ------- | -------- | --------- | -------- |
| NE 50m  | 10.0481  | 76.3306   | ~50m     |
| SE 100m | 10.0474  | 76.3311   | ~100m    |

## Expected Log Output

```
State: WAIT_TARGET → SET_GUIDED (First target received)
Setting DISARM_DELAY=0 for SITL testing
State: SET_GUIDED → ARM (GUIDED mode confirmed)
Streaming setpoints... (2.0s / 3.0s)
ARM command sent
State: ARM → TAKEOFF (Armed)
TAKEOFF to 10.0m
Climbing: 5.7m / 10.0m (target: 8.0m)
Takeoff complete: 8.4m
State: WAIT_TAKEOFF → NAVIGATE (Takeoff complete)
Distance to target: 45.2m
Distance to target: 2.6m
State: NAVIGATE → ARRIVED (At target)
Arrived! Targets completed: 1
State: ARRIVED → WAIT_FOR_NEXT (Published arrival)
Hovering at waypoint - waiting for next geotag... 9s remaining before RTL
Hovering at waypoint - waiting for next geotag... 7s remaining before RTL
10.0s timeout - RTL (resumable if new geotag)
State: WAIT_FOR_NEXT → RTL (Wait timeout)
```

## Testing Checklist

- [x] Waits for FCU connection
- [x] Waits for first target from telem_rx
- [x] Arms on first target
- [x] Takes off to specified altitude
- [x] Navigates to target
- [x] Accepts new targets during NAVIGATE (redirects)
- [x] Publishes arrival status
- [x] Hovers with countdown during WAIT_FOR_NEXT
- [x] RTL if timeout
- [x] Resumes navigation if geotag received during RTL

## Common Issues

| Issue                  | Cause                  | Solution                                        |
| ---------------------- | ---------------------- | ----------------------------------------------- |
| Doesn't arm            | EKF not ready          | Wait 10-15s after SITL start                    |
| Takeoff fails          | GPS fix quality        | Wait for 3D fix                                 |
| Doesn't navigate       | Target not converted   | Check home GPS captured                         |
| Premature RTL          | wait_timeout too short | Increase in config                              |
| No redirect mid-flight | Old code               | Use updated node with all-state target handling |
| Stuck after RTL resume | Mode not GUIDED        | handle_navigate requests GUIDED automatically   |

---

**Last Updated**: December 31, 2024  
**Note**: Unified node with dynamic target acceptance and resumable RTL.
