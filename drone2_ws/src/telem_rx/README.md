# Telemetry RX Node

**Direct Drone-to-Drone Geotag Reception & Dispatch**

## What It Does

Receives disease geotags from Drone-1 via MAVLink telemetry (SiK radio), validates them, and dispatches to the navigation system. This node combines telemetry reception with the validation/dispatch logic, bypassing the GCS entirely.

## Logic Flow

```
MAVLink messages from Telemetry Radio
        │
        ▼
Filter for d_lat, d_lon, d_alt (TYPE_NAMED_VALUE_FLOAT)
        │
        ▼
Buffer incoming values:
├── Store d_lat
├── Store d_lon
└── Store d_alt
        │
        ▼ [all 3 present]
Validate coordinates
        │
        ▼
Dispatch to /drone2/target_position
        │
        ▼
Trigger /drone2/new_target_received
```

## Subscribers

| Topic                      | Type         | Callback             | Description    |
| -------------------------- | ------------ | -------------------- | -------------- |
| `/mavros/debug_value/recv` | `DebugValue` | `mavlink_callback()` | From telemetry |

## Publishers

| Topic                         | Type        | Description       |
| ----------------------------- | ----------- | ----------------- |
| `/drone2/target_position`     | `NavSatFix` | Navigation target |
| `/drone2/new_target_received` | `Bool`      | Mission trigger   |

## Parameters

| Parameter                  | Default   | Description                          |
| -------------------------- | --------- | ------------------------------------ |
| `buffer_timeout_sec`       | `5.0`     | Max time to wait for complete geotag |
| `validate_coordinates`     | `true`    | Enable coordinate validation         |
| `max_distance_from_home_m` | `10000.0` | Max distance from home (meters)      |
| `home_latitude`            | `0.0`     | Home position latitude               |
| `home_longitude`           | `0.0`     | Home position longitude              |
| `override_altitude`        | `false`   | Override received altitude           |
| `target_altitude_m`        | `20.0`    | Target altitude if override enabled  |

## MAVLink Decoding

Buffers 3 `DebugValue` messages with `TYPE_NAMED_VALUE_FLOAT` to reconstruct a complete geotag:

| Name    | Value     | Description   |
| ------- | --------- | ------------- |
| `d_lat` | latitude  | WGS84 degrees |
| `d_lon` | longitude | WGS84 degrees |
| `d_alt` | altitude  | Meters        |

Messages are grouped by header timestamp for synchronization.

## Key Functions

### `mavlink_callback(msg: DebugValue)`

Buffers and reconstructs geotags.

```python
def mavlink_callback(self, msg):
    # Only process NAMED_VALUE_FLOAT type
    if msg.type != DebugValue.TYPE_NAMED_VALUE_FLOAT:
        return

    name = msg.name.strip()

    # Only process disease geotag messages
    if name not in ('d_lat', 'd_lon', 'd_alt'):
        return

    # Store value in buffer
    if name == 'd_lat':
        self.buffer_lat = msg.value_float
    elif name == 'd_lon':
        self.buffer_lon = msg.value_float
    elif name == 'd_alt':
        self.buffer_alt = msg.value_float

    # Dispatch when complete
    if self._buffer_complete():
        self._process_geotag()
        self._clear_buffer()
```

### `_dispatch_target(lat, lon, alt)`

Validates and dispatches to navigation system.

```python
def _dispatch_target(self, lat, lon, alt):
    msg = NavSatFix()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.latitude = lat
    msg.longitude = lon
    msg.altitude = alt if not self.override_alt else self.target_alt

    self.position_pub.publish(msg)

    # Send navigation trigger
    self.trigger_pub.publish(Bool(data=True))
```

## Debugging

### Monitor Reception

```zsh
# Incoming MAVLink messages
ros2 topic echo /mavros/debug_value/recv

# Dispatched targets
ros2 topic echo /drone2/target_position

# Check reception rate
ros2 topic hz /drone2/target_position
```

### Testing Without Hardware

For software-only testing, remap RX to listen to TX's topic:

```zsh
ros2 run telem_rx telem_rx_node --ros-args \
  --remap /mavros/debug_value/recv:=/mavros/debug_value/send
```

### Common Issues

| Issue              | Cause                | Debug                         |
| ------------------ | -------------------- | ----------------------------- |
| No output          | No incoming MAVLink  | Check telemetry link          |
| Incomplete geotags | Message loss         | Check buffer timeout logs     |
| Buffer timeouts    | Telemetry congestion | Increase `buffer_timeout_sec` |
| Rejected targets   | Validation failure   | Check coordinate validity     |

## Integration

This node outputs to `/drone2/target_position` and `/drone2/new_target_received`, which the navigation and mission manager nodes subscribe to. It combines the functionality of the old `telem_rx` and `gcs_to_d2_downlink` nodes.
