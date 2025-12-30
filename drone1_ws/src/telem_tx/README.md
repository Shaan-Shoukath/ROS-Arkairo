# Telemetry TX Node

**Direct Drone-to-Drone Geotag Transmission**

## What It Does

Transmits disease geotags from Drone-1 directly to Drone-2 over MAVLink telemetry (SiK radio), bypassing the Ground Control Station entirely.

## Logic Flow

```
Disease Geotag from Detection Node
        │
        ▼
Encode as NAMED_VALUE_FLOAT:
├── d_lat → latitude
├── d_lon → longitude
└── d_alt → altitude
        │
        ▼
Publish to /mavros/named_value_float/send
        │
        ▼
MAVROS → Autopilot → Telemetry Radio
        │
        ▼
Drone-2 receives via telemetry
```

## Subscribers

| Topic                    | Type              | Callback            | Description    |
| ------------------------ | ----------------- | ------------------- | -------------- |
| `/drone1/disease_geotag` | `GeoPointStamped` | `geotag_callback()` | From detection |

## Publishers

| Topic                            | Type              | Description       |
| -------------------------------- | ----------------- | ----------------- |
| `/mavros/named_value_float/send` | `NamedValueFloat` | MAVLink transport |

## MAVLink Encoding

Each geotag is encoded as 3 sequential NAMED_VALUE_FLOAT messages:

| Name    | Value     | Description   |
| ------- | --------- | ------------- |
| `d_lat` | latitude  | WGS84 degrees |
| `d_lon` | longitude | WGS84 degrees |
| `d_alt` | altitude  | Meters        |

All 3 messages share the same `time_boot_ms` for synchronization on the receiver.

## Key Functions

### `geotag_callback(msg: GeoPointStamped)`

Encodes and transmits geotag over telemetry.

```python
def geotag_callback(self, msg):
    lat = msg.position.latitude
    lon = msg.position.longitude
    alt = msg.position.altitude

    time_boot_ms = int(self.get_clock().now().nanoseconds / 1_000_000)

    # Send latitude
    lat_msg = NamedValueFloat()
    lat_msg.time_boot_ms = time_boot_ms
    lat_msg.name = 'd_lat'
    lat_msg.value = float(lat)
    self.mavlink_pub.publish(lat_msg)

    # Send longitude
    lon_msg = NamedValueFloat()
    lon_msg.time_boot_ms = time_boot_ms
    lon_msg.name = 'd_lon'
    lon_msg.value = float(lon)
    self.mavlink_pub.publish(lon_msg)

    # Send altitude
    alt_msg = NamedValueFloat()
    alt_msg.time_boot_ms = time_boot_ms
    alt_msg.name = 'd_alt'
    alt_msg.value = float(alt)
    self.mavlink_pub.publish(alt_msg)

    self.tx_count += 1
```

## Debugging

### Monitor Transmissions

```zsh
# Outgoing MAVLink messages
ros2 topic echo /mavros/named_value_float/send

# Input geotags
ros2 topic echo /drone1/disease_geotag

# Check transmission rate
ros2 topic hz /mavros/named_value_float/send
```

### Common Issues

| Issue                 | Cause               | Debug                    |
| --------------------- | ------------------- | ------------------------ |
| No transmissions      | No input geotags    | Check detection node     |
| MAVROS errors         | Topic not available | Verify MAVROS is running |
| Drone-2 not receiving | Telemetry link      | Check radio connection   |

## Integration

This node replaces the `d1_to_gcs_uplink` node for direct drone-to-drone communication. The detection node output remains unchanged.
