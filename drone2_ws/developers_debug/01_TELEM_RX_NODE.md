# Telemetry RX Node - Developer Documentation

## Overview

**File**: `drone2_ws/src/telem_rx/telem_rx/telem_rx_node.py`  
**Package**: `telem_rx`  
**Node Name**: `telem_rx_node`  
**Purpose**: Receives disease geotags from Drone-1 via MAVLink telemetry and dispatches to navigation

## What This Node Does

Telemetry receiver and message assembler for Drone-2:

1. **Receives MAVLink messages** from telemetry radio
2. **Assembles GPS coordinates** from 3 separate messages (lat, lon, alt)
3. **Validates coordinates** for reasonableness
4. **Publishes navigation targets** to drone2_navigation
5. **Handles dummy geotags** for testing without Drone-1

## Core Logic & Reasoning

### Message Assembly Buffer

**Problem**: GPS coordinate arrives as 3 separate messages over radio  
**Solution**: Buffering system assembles complete coordinate

```python
# Message 1: d_lat = 10.0478
buffer_lat = 10.0478

# Message 2: d_lon = 76.3303
buffer_lon = 76.3303

# Message 3: d_alt = 50.0
buffer_alt = 50.0
# → Complete! Publish navigation target
```

**Timeout**: If all 3 don't arrive within 5 seconds, buffer resets.

**Why needed?** MAVLink NAMED_VALUE_FLOAT carries single value. GPS needs lat+lon+alt.

### Validation Logic

```python
if distance_from_home(lat, lon) > max_distance:
    reject("Too far from home - likely corrupt data")

if altitude < 0 or altitude > 500:
    reject("Invalid altitude")

if lat == 0.0 and lon == 0.0:
    reject("Null coordinates")
```

**Purpose**: Prevents flying to erroneous GPS coordinates from radio interference.

### Dummy Mode (Testing)

When `use_dummy_geotags=True`:

```python
# Publishes test coordinate at interval
publish_target(dummy_lat, dummy_lon, dummy_alt)
```

**Use Case**: Testing navigation without Drone-1 hardware.

## Subscribers

### `/mavros/debug_value/recv` (mavros_msgs/DebugValue)

- **Source**: MAVROS (from telemetry radio)
- **Purpose**: Receives encoded GPS data from Drone-1
- **Expected Names**: 'd_lat', 'd_lon', 'd_alt'
- **Rate**: Variable (depends on Drone-1 detection rate)

## Publishers

### 1. `/drone2/target_position` (sensor_msgs/NavSatFix)

- **Trigger**: When complete GPS assembled and validated
- **Purpose**: Navigation target for drone2_navigation
- **QoS**: RELIABLE + TRANSIENT_LOCAL
- **Format**: Standard GPS message

### 2. `/drone2/new_target_received` (std_msgs/Bool)

- **Trigger**: Synchronized with target_position
- **Purpose**: Notification flag (legacy, may not be needed)

## Parameters

| Parameter                  | Default | Description                     |
| -------------------------- | ------- | ------------------------------- |
| `buffer_timeout_sec`       | 5.0     | Max time to assemble 3 messages |
| `validate_coordinates`     | True    | Enable validation checks        |
| `max_distance_from_home_m` | 10000.0 | Max valid distance              |
| `home_latitude`            | 0.0     | Reference latitude              |
| `home_longitude`           | 0.0     | Reference longitude             |
| `override_altitude`        | False   | Replace received altitude       |
| `target_altitude_m`        | 20.0    | Override altitude value         |
| **Dummy Mode**             |         |                                 |
| `use_dummy_geotags`        | False   | Enable test mode                |
| `dummy_lat`                | 10.0480 | Test latitude                   |
| `dummy_lon`                | 76.3305 | Test longitude                  |
| `dummy_alt`                | 10.0    | Test altitude                   |
| `dummy_interval_sec`       | 10.0    | Test publish rate               |

## Key Functions

### `mavlink_callback()` - Message Handler

```python
def mavlink_callback(self, msg: DebugValue):
    if msg.name == 'd_lat':
        self.buffer_lat = msg.value_float
    elif msg.name == 'd_lon':
        self.buffer_lon = msg.value_float
    elif msg.name == 'd_alt':
        self.buffer_alt = msg.value_float
        self.try_dispatch()  # Alt is last, try publish
```

**Built-in**: String comparison with `==`

### `try_dispatch()` - Complete Message Check

```python
def try_dispatch(self):
    if all([buffer_lat, buffer_lon, buffer_alt]):
        if validate_coordinates(lat, lon, alt):
            publish_target(lat, lon, alt)
            clear_buffer()
```

**Built-in**: `all()` - Returns True if all items truthy

### `validate_coordinates()` - Sanity Checks

```python
def validate_coordinates(lat, lon, alt) -> bool:
    if not (-90 <= lat <= 90):
        return False
    if not (-180 <= lon <= 180):
        return False
    if distance_from_home(lat, lon) > max_distance:
        return False
    return True
```

### `haversine_distance()` - GPS Distance

```python
def haversine_distance(lat1, lon1, lat2, lon2) -> float:
    R = 6371000  # Earth radius
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat/2)**2 + cos(lat1)*cos(lat2)*sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    return R * c
```

**Built-in**:

- `math.radians()`, `math.sin()`, `math.cos()`
- `math.atan2()`, `math.sqrt()`

## Package Dependencies

### ROS2 Packages

- **rclpy**: Node, QoS, Timer
- **mavros_msgs**: DebugValue
- **sensor_msgs**: NavSatFix
- **std_msgs**: Bool, Header
- **geographic_msgs**: GeoPoint

### Python Libraries

- **math**: Distance calculations
- **typing**: Optional type hints

## Message Flow

```
Drone-1 Radio TX
       ↓
[AIR: MAVLink NAMED_VALUE_FLOAT × 3]
       ↓
Drone-2 Radio RX
       ↓
MAVROS
       ↓
/mavros/debug_value/recv
       ↓
telem_rx_node (this)
       ↓ (assembly + validation)
/drone2/target_position
       ↓
drone2_navigation_node
```

## Error Handling

**Incomplete message**: Buffer times out after 5s, clears  
**Out of range coordinates**: Rejected, logged  
**Radio packet loss**: Waits for retransmission  
**Corrupt data**: Validation catches, prevents bad navigation

## Testing Checklist

- [ ] Node starts and subscribes to MAVLink
- [ ] Receives 'd_lat', 'd_lon', 'd_alt' messages
- [ ] Assembles complete GPS coordinate
- [ ] Validates coordinates correctly
- [ ] Publishes to /drone2/target_position
- [ ] Handles buffer timeout
- [ ] Dummy mode works for testing

## Testing Without Drone-1

**Enable Dummy Mode**:

```yaml
telem_rx:
  ros__parameters:
    use_dummy_geotags: true
    dummy_lat: 10.0480
    dummy_lon: 76.3305
    dummy_alt: 10.0
    dummy_interval_sec: 30.0
```

**Monitor Output**:

```bash
ros2 topic echo /drone2/target_position
```

## Common Issues

**Issue**: No targets received  
**Solution**: Check radio link, verify Drone-1 transmitting

**Issue**: Invalid coordinates rejected  
**Solution**: Check `max_distance_from_home_m` and home position set

**Issue**: Buffer timeout  
**Solution**: Radio link quality issue, messages not arriving together

---

**Last Updated**: December 30, 2025
