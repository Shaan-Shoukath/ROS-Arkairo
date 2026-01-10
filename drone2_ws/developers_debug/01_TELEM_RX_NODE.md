# Telem RX Node - Developer Documentation

## Overview

| Property        | Value                                              |
| --------------- | -------------------------------------------------- |
| **File**        | `drone2_ws/src/telem_rx/telem_rx/telem_rx_node.py` |
| **Package**     | `telem_rx`                                         |
| **Node Name**   | `telem_rx_node`                                    |
| **Config File** | `telem_rx/config/telem_rx_params.yaml`             |
| **Maintainer**  | Shaan Shoukath                                     |

## Purpose

Receives disease geotags from Drone-1 via MAVLink telemetry, validates coordinates, and dispatches to navigation. Supports STATUSTEXT and NAMED_VALUE_FLOAT modes.

---

## Configuration YAML → Code Mapping

```yaml
# telem_rx_params.yaml                    # Python code usage
use_statustext: true              →  self.use_statustext (True=STATUSTEXT, False=NAMED_VALUE_FLOAT)
buffer_timeout_sec: 5.0           →  self.buffer_timeout
validate_coordinates: true        →  self.validate_coords
max_distance_from_home_m: 10000.0 →  self.max_distance
home_latitude: 10.0478            →  self.home_lat
home_longitude: 76.3303           →  self.home_lon
use_dummy_geotags: false          →  self.use_dummy (testing mode)
dummy_lat: 10.0480                →  self.dummy_lat
dummy_lon: 76.3305                →  self.dummy_lon
dummy_interval_sec: 10.0          →  Timer interval for dummy geotag publishing
```

### Parameter Declaration Pattern

```python
# In __init__():
self.declare_parameter('use_statustext', True)    # Declare with default
self.use_statustext = self.get_parameter('use_statustext').value  # Retrieve
```

---

## Key Functions

| Function                   | Purpose                                                 |
| -------------------------- | ------------------------------------------------------- |
| `__init__`                 | Declare parameters, create subscribers/publishers       |
| `statustext_callback`      | Parse `GEOTAG:lat,lon,alt` from STATUSTEXT messages     |
| `debug_callback`           | Buffer NAMED_VALUE_FLOAT messages (d_lat, d_lon, d_alt) |
| `_buffer_complete`         | Check if all 3 components received                      |
| `_process_buffered_geotag` | Validate and dispatch buffered geotag                   |
| `_validate_coordinates`    | Check lat/lon range and distance from home              |
| `_haversine_distance`      | Calculate GPS distance between two points               |
| `_dispatch_target`         | Publish NavSatFix to navigation node                    |
| `publish_dummy_geotag`     | Testing: send fake geotag                               |
| `check_buffer_timeout`     | Clear incomplete buffers after timeout                  |

---

## Subscribers

| Topic                      | Type       | Callback              | Purpose                   |
| -------------------------- | ---------- | --------------------- | ------------------------- |
| `/mavros/statustext/recv`  | StatusText | `statustext_callback` | STATUSTEXT mode (default) |
| `/mavros/debug_value/recv` | DebugValue | `debug_callback`      | NAMED_VALUE_FLOAT mode    |

**QoS**: Uses `BEST_EFFORT` reliability to match MAVROS publisher.

## Publishers

| Topic                         | Type      | Purpose                   |
| ----------------------------- | --------- | ------------------------- |
| `/drone2/target_position`     | NavSatFix | Target GPS for navigation |
| `/drone2/new_target_received` | Bool      | Trigger signal            |

---

## Geotag Formats

**STATUSTEXT mode** (default):

```
GEOTAG:10.0480,76.3305,6.7
```

**NAMED_VALUE_FLOAT mode** (3 separate MAVLink messages):
| MAVLink Name | Field |
|--------------|-------|
| `d_lat` | Latitude |
| `d_lon` | Longitude |
| `d_alt` | Altitude |

---

## Launch Commands

```bash
# With config file
ros2 run telem_rx telem_rx_node --ros-args \
  --params-file ~/Documents/ROS-Arkairo/drone2_ws/src/telem_rx/config/telem_rx_params.yaml

# Dummy mode (testing)
ros2 run telem_rx telem_rx_node --ros-args \
  -p use_dummy_geotags:=true \
  -p dummy_lat:=10.0481 \
  -p dummy_lon:=76.3306
```

---

## Manual Testing

```bash
# Simulate geotag via STATUSTEXT
ros2 topic pub /mavros/statustext/recv mavros_msgs/msg/StatusText \
  "{severity: 6, text: 'GEOTAG:10.0480,76.3305,6.7'}" --once
```

---

**Last Updated**: January 10, 2026
