# GCS Target Receiver Node

**Target Validation and Filtering**

## What It Does

Receives disease geotags from Drone-1 via telemetry uplink. Validates coordinates, filters duplicates, and maintains a persistent log of all validated targets.

## Logic Flow

```
GeoPointStamped from Drone-1
        │
        ▼
Validate Coordinates:
├── Within lat/lon bounds?
├── Not null island (0,0)?
└── Within geofence?
        │
        ▼ [valid]
Duplicate Check:
├── Any target within distance threshold?
└── Any target within time window?
        │
        ▼ [not duplicate]
Assign Unique Target ID
        │
        ▼
Add to Validated List
        │
        ▼
Save to JSON File
        │
        ▼
Forward to Mission Router
```

## Subscribers

| Topic                | Type              | Callback            | Description         |
| -------------------- | ----------------- | ------------------- | ------------------- |
| `/gcs/target_report` | `GeoPointStamped` | `target_callback()` | From Drone-1 uplink |

## Publishers

| Topic                   | Type              | Description          |
| ----------------------- | ----------------- | -------------------- |
| `/gcs/validated_target` | `GeoPointStamped` | To mission router    |
| `/gcs/target_count`     | `Int32`           | Current target count |

## Parameters

| Parameter            | Default        | Description           |
| -------------------- | -------------- | --------------------- |
| `geofence_enabled`   | `true`         | Enable geofence check |
| `geofence_lat_min`   | `37.0`         | Geofence bounds       |
| `geofence_lat_max`   | `38.0`         | Geofence bounds       |
| `geofence_lon_min`   | `-123.0`       | Geofence bounds       |
| `geofence_lon_max`   | `-122.0`       | Geofence bounds       |
| `duplicate_radius_m` | `5.0`          | Distance threshold    |
| `duplicate_time_sec` | `30.0`         | Time threshold        |
| `log_file_path`      | `targets.json` | Persistence file      |

## Key Functions

### `target_callback(msg: GeoPointStamped)`

Main entry point for incoming targets.

```python
def target_callback(self, msg):
    lat = msg.position.latitude
    lon = msg.position.longitude

    # Validate
    if not self._validate_coordinates(lat, lon):
        self.rejected_count += 1
        return

    if not self._check_geofence(lat, lon):
        self.get_logger().warn('Target outside geofence')
        return

    # Duplicate filter
    if self._is_duplicate(lat, lon, msg.header.stamp):
        self.get_logger().info('Duplicate target filtered')
        return

    # Accept target
    target = self._create_target(msg)
    self.validated_targets.append(target)
    self._save_targets()
    self._publish_target(target)
```

### `_check_geofence(lat, lon) -> bool`

Validates target is within operational area.

```python
def _check_geofence(self, lat, lon):
    if not self.geofence_enabled:
        return True

    if lat < self.lat_min or lat > self.lat_max:
        return False
    if lon < self.lon_min or lon > self.lon_max:
        return False

    return True
```

### `_is_duplicate(lat, lon, stamp) -> bool`

Checks for duplicate detections.

```python
def _is_duplicate(self, lat, lon, stamp):
    for target in self.validated_targets:
        # Distance check
        dist = haversine(lat, lon, target.lat, target.lon)
        if dist < self.duplicate_radius:
            # Time check
            time_diff = abs(stamp.sec - target.timestamp)
            if time_diff < self.duplicate_time:
                return True
    return False
```

### `_save_targets()`

Persists targets to JSON file.

```python
def _save_targets(self):
    data = {
        'count': len(self.validated_targets),
        'targets': [
            {
                'id': t.id,
                'lat': t.lat,
                'lon': t.lon,
                'timestamp': t.timestamp,
                'confidence': t.confidence
            }
            for t in self.validated_targets
        ]
    }
    with open(self.log_file, 'w') as f:
        json.dump(data, f, indent=2)
```

## Debugging

### Monitor Targets

```bash
# Incoming targets
ros2 topic echo /gcs/target_report

# Validated targets
ros2 topic echo /gcs/validated_target

# Count
ros2 topic echo /gcs/target_count
```

### View Saved Targets

```bash
cat ~/Documents/ROSArkairo/gcs_ws/targets.json
```

### Common Issues

| Issue           | Cause                  | Debug                |
| --------------- | ---------------------- | -------------------- |
| All rejected    | Geofence wrong         | Check bounds params  |
| Too many dupes  | Radius too large       | Reduce to 2-3m       |
| Missing targets | Time filter too strict | Increase time window |
