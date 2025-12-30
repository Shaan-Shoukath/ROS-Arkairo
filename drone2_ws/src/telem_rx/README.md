# Telemetry RX Node

**Direct Drone-to-Drone Geotag Reception**

## What It Does

Receives disease geotags from Drone-1 via MAVLink telemetry (SiK radio) and reconstructs them for local processing, bypassing the Ground Control Station entirely.

## Logic Flow

```
MAVLink messages from Telemetry Radio
        │
        ▼
Filter for d_lat, d_lon, d_alt
        │
        ▼
Buffer incoming values:
├── Store d_lat
├── Store d_lon
└── Store d_alt
        │
        ▼ [all 3 present]
Reconstruct GeoPointStamped
        │
        ▼
Publish to /drone2/target_geotag
        │
        ▼
Clear buffer
```

## Subscribers

| Topic                       | Type              | Callback             | Description    |
| --------------------------- | ----------------- | -------------------- | -------------- |
| `/mavros/named_value_float` | `NamedValueFloat` | `mavlink_callback()` | From telemetry |

## Publishers

| Topic                   | Type              | Description       |
| ----------------------- | ----------------- | ----------------- |
| `/drone2/target_geotag` | `GeoPointStamped` | Reconstructed GPS |

## Parameters

| Parameter            | Default | Description                          |
| -------------------- | ------- | ------------------------------------ |
| `buffer_timeout_sec` | `5.0`   | Max time to wait for complete geotag |

## MAVLink Decoding

Buffers 3 NAMED_VALUE_FLOAT messages to reconstruct a complete geotag:

| Name    | Value     | Description   |
| ------- | --------- | ------------- |
| `d_lat` | latitude  | WGS84 degrees |
| `d_lon` | longitude | WGS84 degrees |
| `d_alt` | altitude  | Meters        |

Messages are grouped by `time_boot_ms` for synchronization.

## Key Functions

### `mavlink_callback(msg: NamedValueFloat)`

Buffers and reconstructs geotags.

```python
def mavlink_callback(self, msg):
    name = msg.name.strip()

    # Only process disease geotag messages
    if name not in ('d_lat', 'd_lon', 'd_alt'):
        return

    # Check for new message group
    if self.buffer_time_boot_ms != msg.time_boot_ms:
        self._clear_buffer()

    self.buffer_time_boot_ms = msg.time_boot_ms

    # Store value in buffer
    if name == 'd_lat':
        self.buffer_lat = msg.value
    elif name == 'd_lon':
        self.buffer_lon = msg.value
    elif name == 'd_alt':
        self.buffer_alt = msg.value

    # Publish when complete
    if self._buffer_complete():
        self._publish_geotag()
        self._clear_buffer()
```

### `_publish_geotag()`

Reconstructs and publishes the geotag.

```python
def _publish_geotag(self):
    msg = GeoPointStamped()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = 'map'

    msg.position.latitude = self.buffer_lat
    msg.position.longitude = self.buffer_lon
    msg.position.altitude = self.buffer_alt

    self.geotag_pub.publish(msg)
    self.rx_count += 1
```

## Debugging

### Monitor Reception

```zsh
# Incoming MAVLink messages
ros2 topic echo /mavros/named_value_float

# Reconstructed geotags
ros2 topic echo /drone2/target_geotag

# Check reception rate
ros2 topic hz /drone2/target_geotag
```

### Common Issues

| Issue              | Cause                | Debug                         |
| ------------------ | -------------------- | ----------------------------- |
| No output          | No incoming MAVLink  | Check telemetry link          |
| Incomplete geotags | Message loss         | Check buffer timeout logs     |
| Buffer timeouts    | Telemetry congestion | Increase `buffer_timeout_sec` |

## Integration

This node outputs to `/drone2/target_geotag`, which is the same topic the existing `gcs_to_d2_downlink` node subscribes to. No changes to downstream nodes required.
