# GCS to D2 Downlink Node

**Bridge between GCS Telemetry and Drone-2 Local Execution**

## What It Does

Receives target geotags from the Ground Control Station via telemetry and converts them into local navigation commands for Drone-2. Acts as protocol translator between GCS coordinate format and local NavSatFix.

## Logic Flow

```
GeoPointStamped from GCS
        │
        ▼
Validate Coordinates
├── Check lat/lon bounds
├── Reject null island (0,0)
└── Check max distance from home
        │
        ▼ [valid]
Store as Pending Target
        │
        ▼ [auto-dispatch enabled]
Convert to NavSatFix
├── Copy lat/lon
├── Override altitude (configurable)
└── Set status fields
        │
        ▼
Publish to /drone2/target_position
        │
        ▼
Publish trigger to /drone2/new_target_received
```

## Subscribers

| Topic                   | Type              | Callback            | Description          |
| ----------------------- | ----------------- | ------------------- | -------------------- |
| `/drone2/target_geotag` | `GeoPointStamped` | `geotag_callback()` | Target from GCS      |
| `/drone2/mission_start` | `Bool`            | `start_callback()`  | Manual start trigger |

## Publishers

| Topic                         | Type        | Description          |
| ----------------------------- | ----------- | -------------------- |
| `/drone2/target_position`     | `NavSatFix` | Navigation target    |
| `/drone2/new_target_received` | `Bool`      | Trigger for nav node |

## Parameters

| Parameter                  | Default | Description             |
| -------------------------- | ------- | ----------------------- |
| `require_confirmation`     | `false` | Wait for start trigger  |
| `confirmation_timeout_sec` | `5.0`   | Pending target timeout  |
| `override_altitude`        | `true`  | Use fixed altitude      |
| `target_altitude_m`        | `10.0`  | Fixed altitude value    |
| `validate_coordinates`     | `true`  | Enable validation       |
| `max_distance_from_home_m` | `10000` | Max target distance     |
| `home_latitude`            | `0.0`   | Home for distance check |
| `home_longitude`           | `0.0`   | Home for distance check |

## Key Functions

### `geotag_callback(msg: GeoPointStamped)`

Handles incoming target from GCS.

```python
def geotag_callback(self, msg):
    self.total_received += 1
    lat = msg.position.latitude
    lon = msg.position.longitude

    # Validate coordinates
    if self.validate_coords and not self._validate_coordinates(lat, lon):
        self.total_rejected += 1
        self.get_logger().error('Target rejected: invalid coordinates')
        return

    # Auto-dispatch or wait for confirmation
    if not self.require_confirm:
        self._dispatch_target(msg)
    else:
        self.pending_target = msg
        self.pending_time = datetime.now()
```

### `_dispatch_target(geotag: GeoPointStamped)`

Converts and publishes target to navigation.

```python
def _dispatch_target(self, geotag):
    # Create NavSatFix message
    msg = NavSatFix()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.latitude = geotag.position.latitude
    msg.longitude = geotag.position.longitude

    # Override altitude if configured
    if self.override_alt:
        msg.altitude = self.target_alt
    else:
        msg.altitude = geotag.position.altitude

    self.position_pub.publish(msg)

    # Send navigation trigger
    trigger = Bool()
    trigger.data = True
    self.trigger_pub.publish(trigger)

    self.total_dispatched += 1
```

### `_validate_coordinates(lat, lon) -> bool`

Validates GPS coordinates are sane.

```python
def _validate_coordinates(self, lat, lon):
    # Basic bounds
    if lat < -90 or lat > 90: return False
    if lon < -180 or lon > 180: return False

    # Reject null island
    if lat == 0.0 and lon == 0.0: return False

    # Distance check from home
    if self.home_lat != 0.0:
        distance = haversine(lat, lon, self.home_lat, self.home_lon)
        if distance > self.max_distance:
            return False

    return True
```

## Debugging

### Check Targets Arriving

```bash
# Monitor incoming geotags
ros2 topic echo /drone2/target_geotag

# Check dispatch rate
ros2 topic hz /drone2/target_position
```

### Statistics on Shutdown

```
Statistics: received=5, dispatched=4, rejected=1
```

### Common Issues

| Issue              | Cause                       | Debug                      |
| ------------------ | --------------------------- | -------------------------- |
| Targets rejected   | Coordinates invalid         | Check validation params    |
| Nothing dispatched | `require_confirmation=true` | Set to false or send start |
| Wrong altitude     | Override enabled            | Check `target_altitude_m`  |
