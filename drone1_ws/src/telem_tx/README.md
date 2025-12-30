# Telemetry TX Node

**Direct Drone-to-Drone Geotag Transmission**

## What It Does

Transmits disease geotags from Drone-1 directly to Drone-2 over MAVLink telemetry (SiK radio), bypassing the Ground Control Station entirely.

## Logic Flow

```
Disease Geotag from Detection Node
        │
        ▼
Encode as DEBUG_VALUE (TYPE_NAMED_VALUE_FLOAT):
├── d_lat → latitude
├── d_lon → longitude
└── d_alt → altitude
        │
        ▼
Publish to /mavros/debug_value/send
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

| Topic                      | Type         | Description       |
| -------------------------- | ------------ | ----------------- |
| `/mavros/debug_value/send` | `DebugValue` | MAVLink transport |

## MAVLink Encoding

Each geotag is encoded as 3 sequential `DebugValue` messages with `TYPE_NAMED_VALUE_FLOAT`:

| Name    | Value     | Description   |
| ------- | --------- | ------------- |
| `d_lat` | latitude  | WGS84 degrees |
| `d_lon` | longitude | WGS84 degrees |
| `d_alt` | altitude  | Meters        |

All 3 messages share the same header timestamp for synchronization on the receiver.

## Key Functions

### `geotag_callback(msg: GeoPointStamped)`

Encodes and transmits geotag over telemetry.

```python
def geotag_callback(self, msg):
    lat = msg.position.latitude
    lon = msg.position.longitude
    alt = msg.position.altitude

    now = self.get_clock().now().to_msg()

    # Send latitude
    lat_msg = DebugValue()
    lat_msg.header.stamp = now
    lat_msg.header.frame_id = 'map'
    lat_msg.index = -1
    lat_msg.array_id = -1
    lat_msg.name = 'd_lat'
    lat_msg.value_float = float(lat)
    lat_msg.type = DebugValue.TYPE_NAMED_VALUE_FLOAT
    self.mavlink_pub.publish(lat_msg)

    # Send longitude and altitude similarly...
    self.tx_count += 1
```

## Debugging

### Monitor Transmissions

```zsh
# Outgoing MAVLink messages
ros2 topic echo /mavros/debug_value/send

# Input geotags
ros2 topic echo /drone1/disease_geotag

# Check transmission rate
ros2 topic hz /mavros/debug_value/send
```

### Common Issues

| Issue                 | Cause               | Debug                    |
| --------------------- | ------------------- | ------------------------ |
| No transmissions      | No input geotags    | Check detection node     |
| MAVROS errors         | Topic not available | Verify MAVROS is running |
| Drone-2 not receiving | Telemetry link      | Check radio connection   |

## Integration

This node replaces the `d1_to_gcs_uplink` node for direct drone-to-drone communication. The detection node output remains unchanged.
