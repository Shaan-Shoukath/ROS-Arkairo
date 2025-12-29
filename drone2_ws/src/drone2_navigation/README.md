# Drone-2 Navigation Node

**Target Navigation with Arrival Detection**

## What It Does

Navigates Drone-2 to target GPS locations received from the downlink. Continuously publishes position setpoints and detects arrival within configurable radius.

## Logic Flow

```
Target Position Received
        │
        ▼
Store Target + Start Timer
        │
        ▼
Navigation Loop (10Hz):
├── Check Prerequisites (GPS, armed)
├── Calculate Distance to Target (Haversine)
├── If distance < arrival_radius:
│   └── Publish arrival_status = true
│   └── Stop navigating
├── Else:
│   ├── Calculate bearing to target
│   └── Publish GlobalPositionTarget setpoint
└── Check for timeout
```

## Subscribers

| Topic                            | Type        | Callback            | Description   |
| -------------------------------- | ----------- | ------------------- | ------------- |
| `/drone2/target_position`        | `NavSatFix` | `target_callback()` | Target coords |
| `/mavros/global_position/global` | `NavSatFix` | `gps_callback()`    | Current GPS   |
| `/mavros/state`                  | `State`     | `state_callback()`  | FC status     |

## Publishers

| Topic                              | Type                   | Description          |
| ---------------------------------- | ---------------------- | -------------------- |
| `/mavros/setpoint_position/global` | `GlobalPositionTarget` | Position commands    |
| `/drone2/arrival_status`           | `Bool`                 | Arrival notification |

## Parameters

| Parameter                | Default | Description                           |
| ------------------------ | ------- | ------------------------------------- |
| `arrival_radius_m`       | `2.0`   | Distance threshold                    |
| `approach_speed`         | `3.0`   | Navigation speed (not used by MAVROS) |
| `approach_altitude_m`    | `10.0`  | Operating altitude                    |
| `update_rate_hz`         | `10.0`  | Nav loop frequency                    |
| `navigation_timeout_sec` | `120.0` | Max nav time                          |

## Key Functions

### `target_callback(msg: NavSatFix)`

Receives new target and starts navigation.

```python
def target_callback(self, msg):
    self.target_position = msg
    self.navigating = True
    self.arrived = False
    self.nav_start_time = self.get_clock().now()

    self.get_logger().info(
        f'New target: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}'
    )
```

### `navigation_loop()`

Main control loop executed at update_rate_hz.

```python
def navigation_loop(self):
    if not self.navigating:
        return

    if self._check_timeout():
        self.get_logger().error('Navigation timeout')
        self.navigating = False
        return

    distance = self._distance_to_target()

    # Arrival detection
    if distance < self.arrival_radius and not self.arrived:
        self.arrived = True
        self.navigating = False

        msg = Bool()
        msg.data = True
        self.arrival_pub.publish(msg)

        self.get_logger().info(f'Arrived (distance: {distance:.2f}m)')
        return

    # Continue navigation
    self._publish_setpoint()
```

### `_distance_to_target() -> float`

Haversine distance calculation.

```python
def _distance_to_target(self):
    lat1 = radians(self.current_position.latitude)
    lon1 = radians(self.current_position.longitude)
    lat2 = radians(self.target_position.latitude)
    lon2 = radians(self.target_position.longitude)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))

    return 6371000 * c  # Earth radius * angle
```

### `_publish_setpoint()`

Publishes GlobalPositionTarget to MAVROS.

```python
def _publish_setpoint(self):
    msg = GlobalPositionTarget()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT

    # Ignore velocity/acceleration, only use position
    msg.type_mask = (
        IGNORE_VX | IGNORE_VY | IGNORE_VZ |
        IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ |
        IGNORE_YAW_RATE
    )

    msg.latitude = self.target_position.latitude
    msg.longitude = self.target_position.longitude
    msg.altitude = self.target_position.altitude
    msg.yaw = self._calculate_bearing()

    self.setpoint_pub.publish(msg)
```

### `_calculate_bearing() -> float`

Calculates heading from current position to target.

```python
def _calculate_bearing(self):
    lat1, lon1 = radians(current.lat), radians(current.lon)
    lat2, lon2 = radians(target.lat), radians(target.lon)

    dlon = lon2 - lon1
    x = sin(dlon) * cos(lat2)
    y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)

    return atan2(x, y)  # radians
```

## Debugging

### Monitor Navigation

```bash
# Check setpoint publishing rate
ros2 topic hz /mavros/setpoint_position/global

# See current target
ros2 topic echo /drone2/target_position

# Check arrival status
ros2 topic echo /drone2/arrival_status
```

### Common Issues

| Issue         | Cause                      | Debug                    |
| ------------- | -------------------------- | ------------------------ |
| Not moving    | Not in GUIDED mode         | Check `/mavros/state`    |
| Never arrives | `arrival_radius` too small | Increase to 3-5m         |
| Timeout       | Distance too far           | Check target coordinates |
| Wrong heading | Bearing calc issue         | Verify GPS accuracy      |
