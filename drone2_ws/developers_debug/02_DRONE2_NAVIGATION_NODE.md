# Drone2 Navigation Node - Developer Documentation

## Overview

**File**: `drone2_ws/src/drone2_navigation/drone2_navigation/drone2_navigation_node.py`  
**Package**: `drone2_navigation`  
**Node Name**: `drone2_navigation_node`  
**Purpose**: Unified autonomous flight controller for Drone-2 (MERGED with mission_manager functionality)

## What This Node Does

Complete autonomous flight controller:

1. **Waits for target from telem_rx**
2. **Arms motors automatically** on first target
3. **Executes takeoff** to configured altitude
4. **Navigates to GPS target** using position control
5. **Publishes arrival status** when reached
6. **Waits for next target** (30s timeout)
7. **RTL if no new target** received

## Key Feature: UNIFIED ARCHITECTURE

This node **combines** what was previously two nodes:

- **Old**: `drone2_navigation` (no ARM/TAKEOFF) + `mission_manager` (ARM/TAKEOFF but not used)
- **New**: Single `drone2_navigation` node with complete autonomous capability

**Benefit**: Simpler, no coordination between nodes needed.

## State Machine

```
IDLE → WAIT_FCU → WAIT_TARGET → SET_GUIDED → ARM → TAKEOFF →
WAIT_TAKEOFF → NAVIGATE → ARRIVED → WAIT_FOR_NEXT → [NAVIGATE or RTL]
```

**Linear progression** similar to Drone-1 navigation.

## Core Logic

### Target Reception Trigger

```python
def target_callback(self, msg: NavSatFix):
    self.target_gps = msg
    self.target_local = gps_to_local(msg)

    if state == WAIT_TARGET:
        transition_to(SET_GUIDED)  # First target → start flight
    elif state == WAIT_FOR_NEXT:
        transition_to(NAVIGATE)    # New target → continue mission
```

### GPS to Local Conversion

```python
def gps_to_local(lat, lon, alt):
    # Convert GPS to local ENU coordinates
    dlat = (lat - home_lat) * 111132.92  # meters per degree
    dlon = (lon - home_lon) * 111132.92 * cos(home_lat)
    x = dlon + home_x
    y = dlat + home_y
    z = alt
    return (x, y, z)
```

**Purpose**: ArduPilot position controller uses local frame.

### Arrival Detection

```python
distance = sqrt((target_x - current_x)² + (target_y - current_y)²)
if distance < arrival_radius:
    publish_arrival()
    start_wait_timer()
```

## Subscribers

### 1. `/drone2/target_position` (sensor_msgs/NavSatFix)

- **Source**: Telemetry RX Node
- **Purpose**: GPS targets from Drone-1
- **Trigger**: ARM on first, NAVIGATE on subsequent

### 2. `/mavros/state` (mavros_msgs/State)

- **Source**: MAVROS
- **Purpose**: FCU connection and mode monitoring
- **Usage**: Check connected, armed, mode

### 3. `/mavros/global_position/global` (sensor_msgs/NavSatFix)

- **Source**: MAVROS
- **Purpose**: Current GPS, home position capture

### 4. `/mavros/local_position/pose` (geometry_msgs/PoseStamped)

- **Source**: MAVROS
- **Purpose**: Local position for navigation

## Publishers

### 1. `/mavros/setpoint_position/local` (geometry_msgs/PoseStamped)

- **Rate**: 10Hz continuous
- **Purpose**: Position setpoints to ArduPilot
- **Critical**: Must stream or failsafe triggers

### 2. `/drone2/arrival_status` (std_msgs/Bool)

- **Trigger**: When arrived at target
- **Purpose**: Triggers detection/centering node
- **QoS**: RELIABLE

### 3. `/drone2/navigation_status` (std_msgs/String)

- **Rate**: 1Hz
- **Purpose**: Current state for monitoring

## Parameters

| Parameter             | Default | Description                  |
| --------------------- | ------- | ---------------------------- |
| `takeoff_altitude_m`  | 10.0    | Takeoff height               |
| `arrival_radius_m`    | 3.0     | Distance to consider arrived |
| `setpoint_rate_hz`    | 10.0    | Position command rate        |
| `wait_timeout_sec`    | 30.0    | Time to wait for next target |
| `takeoff_timeout_sec` | 60.0    | Max takeoff time             |
| `fcu_timeout_sec`     | 30.0    | FCU connection timeout       |

## Key Functions

### `fsm_update()` - State Machine (2Hz)

Handles state transitions based on current state.

### `handle_arm()` - Arming Sequence

1. Switch to GUIDED mode
2. Stream setpoints for 3 seconds
3. Send ARM command
4. Retry until armed

### `handle_takeoff()` - Takeoff Command

```python
req = CommandTOL.Request()
req.altitude = takeoff_alt
req.latitude = home_lat
req.longitude = home_lon
takeoff_client.call_async(req)
```

### `handle_navigate()` - Position Control

Continuously publishes target position as setpoint.

### `publish_setpoint()` - 10Hz Streaming

Runs continuously, publishes either:

- Hold position (during ARM)
- Target position (during NAVIGATE)

## Service Clients

- `/mavros/cmd/arming` - CommandBool
- `/mavros/set_mode` - SetMode
- `/mavros/cmd/takeoff` - CommandTOL

## Package Dependencies

### ROS2

- rclpy, std_msgs, sensor_msgs, geometry_msgs, mavros_msgs

### Python

- math (sin, cos, sqrt, radians)
- enum (Enum, auto)
- typing (Optional, Tuple)

## Critical Design Rules

1. **Setpoints stream at 10Hz minimum** from ARM onwards
2. **ARM once per mission**
3. **TAKEOFF uses MAV_CMD_NAV_TAKEOFF**
4. **GUIDED mode only**
5. **Wait window after arrival** for spray operations

## Testing Checklist

- [ ] Waits for FCU connection
- [ ] Waits for first target from telem_rx
- [ ] Arms on first target
- [ ] Takes off to specified altitude
- [ ] Navigates to target
- [ ] Publishes arrival status
- [ ] Waits 30s for next target
- [ ] RTL if timeout

## Common Issues

**Doesn't arm**: Check setpoint streaming, FCU mode  
**Takeoff fails**: Check GPS fix quality  
**Doesn't navigate**: Check target received, GPS conversion  
**Premature RTL**: Check wait_timeout parameter

---

**Last Updated**: December 30, 2025  
**Note**: This unified node includes all autonomous flight functionality.
