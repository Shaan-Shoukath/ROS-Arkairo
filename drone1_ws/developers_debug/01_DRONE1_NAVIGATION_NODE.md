# Drone1 Navigation Node - Developer Documentation

## Overview

**File**: `drone1_ws/src/drone1_navigation/drone1_navigation/drone1_navigation_node.py`  
**Package**: `drone1_navigation`  
**Node Name**: `drone1_navigation_node`  
**Purpose**: Autonomous flight controller for Drone-1 survey missions

## What This Node Does

The Drone1 Navigation Node is the brain of the autonomous survey drone. It:

1. **Receives waypoint missions** from the KML Lane Planner
2. **Manages the complete flight lifecycle** (ARM → TAKEOFF → NAVIGATE → RTL)
3. **Streams position setpoints** to MAVROS at 10Hz (required by ArduPilot)
4. **Follows survey lanes** using path-following logic with lookahead
5. **Controls disease detection** (enables only during active flight)
6. **Handles all MAVROS service calls** (arming, mode changes, takeoff)

## Core Logic & Reasoning

### State Machine Architecture

The node uses a **linear state machine** that progresses through these states:

```
IDLE → WAIT_FOR_FCU → WAIT_FOR_MISSION → SET_GUIDED → ARM →
TAKEOFF_CMD → WAIT_FOR_TAKEOFF → NAVIGATE → MISSION_COMPLETE → RTL
```

**Key Design Decisions:**

- **Linear progression**: No loops or backward transitions (ensures predictable behavior)
- **One-time ARM**: Drone arms exactly once per mission
- **One-time TAKEOFF**: Uses `MAV_CMD_NAV_TAKEOFF` command (ArduPilot standard)
- **Continuous setpoints**: Streams at 10Hz from ARM onwards (ArduPilot requirement)
- **GUIDED mode only**: No AUTO mode, no RC override

### Path Following Algorithm

The navigation uses a **lookahead-based path follower**:

```python
# From current position, look ahead on the path
lookahead_point = find_point_at_distance(current_pos, path, lookahead_distance)
setpoint = lookahead_point
```

**Why this approach?**

- Smooth turns without sharp corners
- Natural speed adaptation on curves
- Robust to GPS noise and wind drift
- Simple and predictable behavior

### Detection Enable Logic

Detection is enabled **ONLY during NAVIGATE state**:

```python
if state == FlightState.NAVIGATE:
    enable_detection = True
else:
    enable_detection = False
```

**Reasoning**: Prevents false detections during takeoff, landing, and transitions.

## Subscribers

### 1. `/mission/lane_segments` (drone1_msgs/LaneSegmentArray)

- **Source**: KML Lane Planner Node
- **Purpose**: Receives survey waypoints organized into parallel lanes
- **Trigger**: Transitions from WAIT_FOR_MISSION → SET_GUIDED
- **Data Structure**: Array of LaneSegments, each containing multiple waypoints

### 2. `/mavros/state` (mavros_msgs/State)

- **Source**: MAVROS
- **Purpose**: Monitors FCU connection status and flight mode
- **Critical Fields**:
  - `connected`: FCU heartbeat status
  - `armed`: Motor arming status
  - `mode`: Current flight mode (e.g., "GUIDED", "RTL")
- **Usage**: State machine transitions depend on this data

### 3. `/mavros/global_position/relative_alt` (std_msgs/Float64)

- **Source**: MAVROS (from ArduPilot)
- **Purpose**: Altitude above home position for takeoff monitoring
- **Usage**: Determines when drone has reached takeoff altitude

### 4. `/mavros/local_position/pose` (geometry_msgs/PoseStamped)

- **Source**: MAVROS (from ArduPilot EKF)
- **Purpose**: Local position in ENU frame for navigation
- **Coordinate System**: East-North-Up (ENU) relative to home
- **Usage**:
  - Path following distance calculations
  - Current position for setpoint generation

## Publishers

### 1. `/mavros/setpoint_position/local` (geometry_msgs/PoseStamped)

- **Rate**: 10Hz (continuous during flight)
- **Purpose**: Position setpoints for ArduPilot position controller
- **Frame**: Local ENU (map frame)
- **Critical**: Must stream continuously or ArduPilot will trigger failsafe

### 2. `/drone1/next_waypoint` (sensor_msgs/NavSatFix)

- **Rate**: On waypoint change
- **Purpose**: Current navigation target for monitoring and GCS display
- **Format**: GPS coordinates (latitude, longitude, altitude)

### 3. `/drone1/navigation_status` (std_msgs/String)

- **Rate**: 1Hz
- **Purpose**: Current FSM state for monitoring
- **Values**: State names (e.g., "NAVIGATE", "WAIT_FOR_TAKEOFF")

### 4. `/drone1/detection_enable` (std_msgs/Bool)

- **Rate**: On state change
- **Purpose**: Enable/disable disease detection
- **Logic**: True only during NAVIGATE state

## Service Clients (MAVROS)

### 1. `/mavros/cmd/arming` (mavros_msgs/CommandBool)

- **Called**: During ARM state
- **Purpose**: Arm/disarm motors
- **Retry Logic**: Attempts every 2 seconds until success

### 2. `/mavros/set_mode` (mavros_msgs/SetMode)

- **Called**: Multiple states (SET_GUIDED, RTL)
- **Purpose**: Change flight mode
- **Modes Used**: "GUIDED", "RTL"

### 3. `/mavros/cmd/takeoff` (mavros_msgs/CommandTOL)

- **Called**: During TAKEOFF_CMD state (once)
- **Purpose**: Execute MAV_CMD_NAV_TAKEOFF
- **Parameters**:
  - Altitude: From `takeoff_altitude_m` parameter
  - Latitude/Longitude: Current GPS position
  - Yaw: NaN (no heading constraint)

## Parameters

| Parameter                   | Default | Description                           |
| --------------------------- | ------- | ------------------------------------- |
| `takeoff_altitude_m`        | 50.0    | Altitude for takeoff (meters AGL)     |
| `navigation_altitude_m`     | 50.0    | Survey flight altitude                |
| `waypoint_arrival_radius_m` | 3.0     | Distance to consider waypoint reached |
| `path_lookahead_m`          | 12.0    | Lookahead distance for path following |
| `path_max_target_step_m`    | 25.0    | Max distance jump for setpoint        |
| `path_end_radius_m`         | 5.0     | Distance to consider lane complete    |
| `setpoint_rate_hz`          | 10.0    | Position setpoint publishing rate     |
| `takeoff_timeout_sec`       | 90.0    | Max time to reach takeoff altitude    |
| `fcu_timeout_sec`           | 30.0    | Max time to wait for FCU connection   |

## Key Functions Explained

### `fsm_update()` - State Machine Tick

```python
def fsm_update(self):
    """Main state machine update (2Hz)"""
```

**Purpose**: Called every 0.5 seconds to progress through flight states  
**Logic**: Uses if/elif chain to handle current state  
**Why 2Hz?**: Balance between responsiveness and CPU usage

### `publish_setpoint()` - Setpoint Streaming

```python
def publish_setpoint(self):
    """Publish position setpoints (10Hz)"""
```

**Purpose**: Streams position commands to MAVROS  
**Critical**: Must run at 10Hz minimum or ArduPilot triggers failsafe  
**Built-in Function**: `self.get_clock().now().to_msg()` - Gets ROS timestamp

### `navigate_to_target()` - Path Following

```python
def navigate_to_target(self, target: Tuple[float, float, float]) -> PoseStamped:
```

**Purpose**: Generates setpoint using lookahead algorithm  
**Math**: Uses vector projection and distance calculations  
**Returns**: PoseStamped message ready for publishing

### `haversine_distance()` - GPS Distance Calculation

```python
def haversine_distance(lat1, lon1, lat2, lon2) -> float:
```

**Purpose**: Calculates great-circle distance between two GPS points  
**Formula**: Haversine formula for spherical Earth  
**Returns**: Distance in meters  
**Built-in Functions**:

- `math.radians()` - Converts degrees to radians
- `math.sin()`, `math.cos()` - Trigonometric functions
- `math.atan2()` - Two-argument arctangent
- `math.sqrt()` - Square root

## Package Dependencies

### ROS2 Packages

- **rclpy**: ROS2 Python client library
  - `Node` - Base class for ROS nodes
  - `QoSProfile` - Quality of Service configuration
  - `ReliabilityPolicy`, `DurabilityPolicy` - QoS policies

### Message Types

- **std_msgs**: Standard message types
  - `String` - Text messages
  - `Bool` - Boolean values
  - `Float64` - Double precision floats
- **sensor_msgs**: Sensor data messages
  - `NavSatFix` - GPS data (lat/lon/alt)
- **geometry_msgs**: Geometric data
  - `PoseStamped` - Position + orientation with timestamp
- **mavros_msgs**: MAVROS-specific messages
  - `State` - FCU connection status
  - Services: `CommandBool`, `SetMode`, `CommandTOL`
- **drone1_msgs**: Custom messages
  - `LaneSegmentArray` - Survey lane waypoints

### Python Libraries

- **math**: Mathematical functions (sin, cos, sqrt, atan2, radians, degrees)
- **enum**: Enum type for state machine (Enum, auto)
- **typing**: Type hints (Optional, List, Tuple)
- **datetime**: Timestamp handling

## Critical Design Rules

1. **Position setpoints MUST stream at 10Hz minimum** during flight
2. **ARM happens exactly once** - no re-arming mid-flight
3. **TAKEOFF uses MAV_CMD_NAV_TAKEOFF** - not velocity commands
4. **GUIDED mode only** - no switching to AUTO or other modes
5. **Detection enabled only during NAVIGATE** - prevents false positives
6. **Linear state progression** - no loops or backward transitions
7. **Throttle/PWM controlled by ArduPilot** - node only sends position commands

## Error Handling

- **FCU timeout**: Waits `fcu_timeout_sec` for connection
- **Takeoff timeout**: Waits `takeoff_timeout_sec` for altitude
- **ARM failure**: Retries every 2 seconds
- **Mode change failure**: Retries on next FSM tick
- **Disarm during flight**: Immediately transitions to RTL

## Testing Checklist

- [ ] Node starts and waits for FCU connection
- [ ] Receives mission and transitions to SET_GUIDED
- [ ] Successfully arms motors
- [ ] Executes takeoff to specified altitude
- [ ] Follows survey lanes with smooth path following
- [ ] Detection enabled during NAVIGATE only
- [ ] Completes mission and executes RTL
- [ ] Setpoints stream continuously at 10Hz
- [ ] Handles FCU disconnection gracefully

## Common Issues & Solutions

**Issue**: Drone doesn't arm  
**Solution**: Check setpoint streaming started, FCU in GUIDED mode, and pre-arm checks passed

**Issue**: Jerky navigation  
**Solution**: Increase `path_lookahead_m` parameter for smoother turns

**Issue**: Waypoints skipped  
**Solution**: Decrease `waypoint_arrival_radius_m` for tighter following

**Issue**: Takeoff timeout  
**Solution**: Check GPS fix quality and barometer calibration

## SITL Simulation Testing

### Quick Start (Simulation Mode)

```bash
# Terminal 1: Start ArduPilot SITL
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map -l 10.0478,76.3303,0,0 -w

# Terminal 2: MAVROS
ros2 launch mavros apm.launch.py fcu_url:=udp://:14550@127.0.0.1:14555

# Terminal 3: Drone1 full stack
cd ~/Documents/ROSArkairo/drone1_ws && source install/setup.zsh
ros2 launch drone1_bringup drone1_survey.launch.py
```

### Test Locations from Home (10.0478, 76.3303)

| Target | Latitude | Longitude | Distance |
| ------ | -------- | --------- | -------- |
| NE 50m | 10.0482  | 76.3307   | ~50m     |
| E 100m | 10.0478  | 76.3312   | ~100m    |

### Key SITL Parameters

For testing, the node automatically sets:

- `DISARM_DELAY=0` - Prevents auto-disarm during development

---

**Maintained by**: Shaan Shoukath  
**Last Updated**: December 31, 2025
