# Telem RX Node - Developer Documentation

## Overview

| Property        | Value                                              |
| --------------- | -------------------------------------------------- |
| **File**        | `drone2_ws/src/telem_rx/telem_rx/telem_rx_node.py` |
| **Package**     | `telem_rx`                                         |
| **Node Name**   | `telem_rx_node`                                    |
| **Config File** | `telem_rx/config/telem_rx_params.yaml`             |
| **Maintainer**  | Shaan Shoukath                                     |

---

## Purpose

Receives disease geotags from Drone 1 via MAVLink telemetry (through GCS forwarder), validates coordinates, and dispatches to drone2_navigation. Supports STATUSTEXT and NAMED_VALUE_FLOAT modes.

---

## Hardware Setup

```
┌─────────────────────────────────────────────────────────────┐
│                       DRONE 2                               │
│                                                             │
│   ┌─────────────────┐                                       │
│   │ Telemetry Radio │ ~~~ RF (Net ID: 25) ~~~ GCS           │
│   └────────┬────────┘                                       │
│            │                                                │
│            ▼                                                │
│   ┌─────────────────┐                                       │
│   │  Cube Orange+   │                                       │
│   │                 │                                       │
│   │  SERIAL1/TELEM1 │ ← From Radio (MAVLink2, 57600)        │
│   │  SERIAL0 (USB)  │ → To Pi (MAVLink2, 115200)            │
│   └────────┬────────┘                                       │
│            │ USB Cable                                      │
│            ▼                                                │
│   ┌─────────────────┐                                       │
│   │  Raspberry Pi 5 │                                       │
│   │  (Companion)    │                                       │
│   │                 │                                       │
│   │  telem_rx_node  │ ← /mavros/statustext/recv             │
│   └─────────────────┘                                       │
└─────────────────────────────────────────────────────────────┘
```

### ArduPilot Parameters (Drone 2 Cube)

```
SYSID_THISMAV = 2        # Drone 2 system ID
SERIAL0_PROTOCOL = 2     # USB = MAVLink2
SERIAL0_BAUD = 115       # 115200
SERIAL0_OPTIONS = 0      # Forwarding enabled
SERIAL1_PROTOCOL = 2     # TELEM1 = MAVLink2
SERIAL1_BAUD = 57        # 57600
SERIAL1_OPTIONS = 0      # Forwarding enabled
```

---

## State Variables

| Variable               | Type    | Purpose                              |
| ---------------------- | ------- | ------------------------------------ |
| `use_statustext`       | `bool`  | True=STATUSTEXT mode (default)       |
| `validate_coords`      | `bool`  | Enable coordinate validation         |
| `home_lat`, `home_lon` | `float` | Reference for distance validation    |
| `max_distance`         | `float` | Max allowed distance from home (m)   |
| `last_geotag`          | `str`   | For duplicate detection              |
| `rx_count`             | `int`   | Total geotags received               |
| `rejected_count`       | `int`   | Invalid geotags rejected             |
| `duplicate_count`      | `int`   | Duplicates ignored                   |
| `use_dummy`            | `bool`  | Enable dummy geotag mode for testing |

---

## Configuration YAML → Code Mapping

```yaml
# telem_rx_params.yaml                    # Python code usage
use_statustext: true              →  self.use_statustext
buffer_timeout_sec: 5.0           →  self.buffer_timeout
validate_coordinates: true        →  self.validate_coords
max_distance_from_home_m: 10000.0 →  self.max_distance
home_latitude: 28.4215            →  self.home_lat
home_longitude: 77.5243           →  self.home_lon
use_dummy_geotags: false          →  self.use_dummy
dummy_lat: 28.4223                →  self.dummy_lat
dummy_lon: 77.5249                →  self.dummy_lon
dummy_alt: 6.7                    →  self.dummy_alt
dummy_interval_sec: 10.0          →  Timer interval
```

---

## Key Functions and Why They Exist

| Function                | Why It Exists                                    | Variables Used                             |
| ----------------------- | ------------------------------------------------ | ------------------------------------------ |
| `__init__`              | Initialize node, declare params, create sub/pubs | All params, `statustext_sub`, `target_pub` |
| `statustext_callback`   | Parse `GEOTAG:` messages from STATUSTEXT         | `last_geotag`, `rx_count`                  |
| `debug_callback`        | Buffer NAMED_VALUE_FLOAT messages                | `geotag_buffer`                            |
| `_validate_coordinates` | Check lat/lon range and distance from home       | `home_lat`, `home_lon`, `max_distance`     |
| `_haversine_distance`   | Calculate GPS distance between two points        | -                                          |
| `_dispatch_target`      | Publish NavSatFix to navigation node             | `target_pub`                               |
| `publish_dummy_geotag`  | Testing: send fake geotag                        | `dummy_lat`, `dummy_lon`                   |

---

## Subscribers

| Topic                      | Type         | Callback              | QoS         | Purpose                |
| -------------------------- | ------------ | --------------------- | ----------- | ---------------------- |
| `/mavros/statustext/recv`  | `StatusText` | `statustext_callback` | BEST_EFFORT | STATUSTEXT mode        |
| `/mavros/debug_value/recv` | `DebugValue` | `debug_callback`      | BEST_EFFORT | NAMED_VALUE_FLOAT mode |

## Publishers

| Topic                         | Type        | Purpose                   |
| ----------------------------- | ----------- | ------------------------- |
| `/drone2/target_position`     | `NavSatFix` | Target GPS for navigation |
| `/drone2/new_target_received` | `Bool`      | Trigger signal            |

---

## Data Flow

```
GCS Forwarder (Laptop)
     │
     │ Radio B ~~~ RF ~~~
     │
     ▼
Drone 2 Radio (TELEM1)
     │
     │ MAVLink STATUSTEXT
     ▼
Cube Orange+ SERIAL1
     │
     │ (ArduPilot auto-forwards to SERIAL0)
     ▼
Cube Orange+ USB (SERIAL0)
     │
     │ USB Cable
     ▼
MAVROS (Drone 2)
     │
     │ /mavros/statustext/recv
     ▼
┌─────────────────────────┐
│     telem_rx_node       │
│                         │
│  • Parse "GEOTAG:..."   │
│  • Validate coords      │
│  • Dispatch NavSatFix   │
└─────────────┬───────────┘
              │ NavSatFix
              │ /drone2/target_position
              ▼
    drone2_navigation_node
```

---

## Geotag Parsing

### STATUSTEXT mode

```python
text = "GEOTAG:28.4223,77.5249,6.7"
coords = text[7:].split(',')  # Remove "GEOTAG:" prefix
lat = float(coords[0])  # 28.4223
lon = float(coords[1])  # 77.5249
alt = float(coords[2])  # 6.7
```

### Validation Checks

1. **Lat range**: -90 to +90
2. **Lon range**: -180 to +180
3. **Distance from home**: < 10km (configurable)

---

## Manual Testing

### Start MAVROS (Hardware - USB)

```bash
ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200
```

### Run telem_rx

```bash
cd ~/Documents/ROS-Arkairo/drone2_ws && source install/setup.zsh
ros2 run telem_rx telem_rx_node --ros-args \
  --params-file ~/Documents/ROS-Arkairo/drone2_ws/src/telem_rx/config/telem_rx_params.yaml
```

### Simulate Geotag (for testing without Drone 1)

```bash
ros2 topic pub /mavros/statustext/recv mavros_msgs/msg/StatusText \
  "{severity: 6, text: 'GEOTAG:28.4223,77.5249,6.7'}" --once
```

### Watch Output

```bash
ros2 topic echo /drone2/target_position
```

### Expected Output

```yaml
latitude: 28.4223
longitude: 77.5249
altitude: 6.7
```

### Dummy Mode (Testing without Radio)

```bash
ros2 run telem_rx telem_rx_node --ros-args \
  -p use_dummy_geotags:=true \
  -p dummy_lat:=28.4223 \
  -p dummy_lon:=77.5249
```

---

**Last Updated**: January 13, 2026
