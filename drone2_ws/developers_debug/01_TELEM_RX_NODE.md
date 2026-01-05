# Telem RX Node - Developer Documentation

## Overview

**File**: `drone2_ws/src/telem_rx/telem_rx/telem_rx_node.py`  
**Package**: `telem_rx`  
**Node Name**: `telem_rx_node`  
**Author**: Shaan Shoukath

## Purpose

Receives disease geotags from Drone-1 via MAVLink telemetry, validates coordinates, and dispatches to navigation for spraying. Direct drone-to-drone communication without GCS.

---

## Hardware Preconditions

```
Raspberry Pi 5 (companion) ─── UART ───► Cube Orange+ TELEM2
Cube Orange+ TELEM1 ───────────────────► Telemetry Radio (AIR)
```

## ArduPilot Parameters (MUST SET)

```
SERIAL1_PROTOCOL = 2     # TELEM1 = MAVLink2 (radio)
SERIAL1_BAUD = 57        # 57600
SERIAL2_PROTOCOL = 2     # TELEM2 = MAVLink2 (Pi)
SERIAL2_BAUD = 921       # 921600
MAV_FORWARD = 1          # CRITICAL: Forward messages between ports
SYSID_THISMAV = 2        # Drone 2 system ID
```

## Telemetry Radio Settings

```
Baud Rate: 57600
Net ID: Same as Drone 1
TX Power: 20 dBm
Air Speed: 64 kbps
```

---

## Subscribers

| Topic                      | Type       | Purpose              |
| -------------------------- | ---------- | -------------------- |
| `/mavros/debug_value/recv` | DebugValue | MAVLink from Drone 1 |

## Publishers

| Topic                         | Type      | Purpose               |
| ----------------------------- | --------- | --------------------- |
| `/drone2/target_position`     | NavSatFix | Target for navigation |
| `/drone2/new_target_received` | Bool      | Trigger signal        |

## MAVLink Decoding

| MAVLink Name | Field     | Description     |
| ------------ | --------- | --------------- |
| `d_lat`      | Latitude  | GPS latitude    |
| `d_lon`      | Longitude | GPS longitude   |
| `d_alt`      | Altitude  | Target altitude |

---

## Key Parameters

```yaml
buffer_timeout_sec: 5.0 # Timeout for incomplete messages
validate_coordinates: true # Enable GPS validation
max_distance_from_home_m: 10000.0
home_latitude: 10.0478
home_longitude: 76.3303
use_dummy_geotags: false # Enable for testing without Drone 1
```

---

## Data Flow

```
Drone 1 Radio ~~~ RF ~~~ Radio → Cube TELEM1
    → Cube TELEM2 → MAVROS → /mavros/debug_value/recv
    → telem_rx → /drone2/target_position → Navigation
```

---

## Launch Commands

```bash
# MAVROS
ros2 launch mavros apm.launch.py fcu_url:=serial:///dev/ttyAMA0:921600

# Telem RX (normal mode)
ros2 run telem_rx telem_rx_node --ros-args \
  --params-file src/telem_rx/config/telem_rx_params.yaml

# Telem RX (dummy mode - for testing without Drone 1)
ros2 run telem_rx telem_rx_node --ros-args \
  -p use_dummy_geotags:=true \
  -p dummy_lat:=10.0481 \
  -p dummy_lon:=76.3306
```

---

**Last Updated**: January 5, 2026
