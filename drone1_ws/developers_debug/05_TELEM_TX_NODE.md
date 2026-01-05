# Telem TX Node - Developer Documentation

## Overview

**File**: `drone1_ws/src/telem_tx/telem_tx/telem_tx_node.py`  
**Package**: `telem_tx`  
**Node Name**: `telem_tx_node`  
**Author**: Shaan Shoukath

## Purpose

Transmits disease geotags from Drone-1 to Drone-2 via MAVLink telemetry radio. Encodes GPS coordinates as NAMED_VALUE_FLOAT messages for direct drone-to-drone communication without GCS.

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
SYSID_THISMAV = 1        # Drone 1 system ID
```

## Telemetry Radio Settings

```
Baud Rate: 57600
Net ID: Same as Drone 2
TX Power: 20 dBm
Air Speed: 64 kbps
```

---

## Subscribers

| Topic                    | Type            | Purpose                    |
| ------------------------ | --------------- | -------------------------- |
| `/drone1/disease_geotag` | GeoPointStamped | Disease GPS from detection |

## Publishers

| Topic                      | Type       | Purpose              |
| -------------------------- | ---------- | -------------------- |
| `/mavros/debug_value/send` | DebugValue | MAVLink transmission |

## MAVLink Encoding

| Field     | MAVLink Name | Type              |
| --------- | ------------ | ----------------- |
| Latitude  | `d_lat`      | NAMED_VALUE_FLOAT |
| Longitude | `d_lon`      | NAMED_VALUE_FLOAT |
| Altitude  | `d_alt`      | NAMED_VALUE_FLOAT |

---

## Data Flow

```
Detection Node → /drone1/disease_geotag → telem_tx
    → /mavros/debug_value/send → MAVROS → Cube TELEM2
    → Cube TELEM1 → Radio ~~~ RF ~~~ Drone 2 Radio
```

---

## Launch Command

```bash
# UART connection
ros2 launch mavros apm.launch.py fcu_url:=serial:///dev/ttyAMA0:921600

# Run node
ros2 run telem_tx telem_tx_node
```

---

**Last Updated**: January 5, 2026
