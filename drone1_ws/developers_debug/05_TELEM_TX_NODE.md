# Telem TX Node - Developer Documentation

## Overview

| Property        | Value                                              |
| --------------- | -------------------------------------------------- |
| **File**        | `drone1_ws/src/telem_tx/telem_tx/telem_tx_node.py` |
| **Package**     | `telem_tx`                                         |
| **Node Name**   | `telem_tx_node`                                    |
| **Config File** | None (uses ROS parameters)                         |
| **Maintainer**  | Shaan Shoukath                                     |

---

## Purpose

Transmits disease geotags from Drone 1 to GCS/Drone 2 via MAVLink telemetry radio. Encodes GPS coordinates as **STATUSTEXT** messages with format `GEOTAG:lat,lon,alt`.

---

## Hardware Setup

```
┌─────────────────────────────────────────────────────────────┐
│                       DRONE 1                               │
│                                                             │
│   ┌─────────────────┐                                       │
│   │  Raspberry Pi 5 │                                       │
│   │  (Companion)    │                                       │
│   │                 │                                       │
│   │  telem_tx_node  │ → /mavros/statustext/send             │
│   └────────┬────────┘                                       │
│            │ USB Cable                                      │
│            ▼                                                │
│   ┌─────────────────┐                                       │
│   │  Cube Orange+   │                                       │
│   │                 │                                       │
│   │  SERIAL0 (USB)  │ ← From Pi (MAVLink2, 115200)          │
│   │  SERIAL1/TELEM1 │ → To Radio (MAVLink2, 57600)          │
│   └────────┬────────┘                                       │
│            │                                                │
│            ▼                                                │
│   ┌─────────────────┐                                       │
│   │ Telemetry Radio │ ~~~ RF (Net ID: 25) ~~~ GCS/Drone 2   │
│   └─────────────────┘                                       │
└─────────────────────────────────────────────────────────────┘
```

### ArduPilot Parameters (Drone 1 Cube)

```
SYSID_THISMAV = 1        # Drone 1 system ID
SERIAL0_PROTOCOL = 2     # USB = MAVLink2
SERIAL0_BAUD = 115       # 115200
SERIAL0_OPTIONS = 0      # Forwarding enabled
SERIAL1_PROTOCOL = 2     # TELEM1 = MAVLink2
SERIAL1_BAUD = 57        # 57600
SERIAL1_OPTIONS = 0      # Forwarding enabled
```

---

## State Variables

| Variable         | Type           | Purpose                                                           |
| ---------------- | -------------- | ----------------------------------------------------------------- |
| `use_statustext` | `bool`         | Encoding mode: True=STATUSTEXT (default), False=NAMED_VALUE_FLOAT |
| `tx_count`       | `int`          | Total geotags transmitted                                         |
| `GEOTAG_PREFIX`  | `str`          | Constant: `"GEOTAG:"`                                             |
| `geotag_sub`     | `Subscription` | Subscriber to disease geotag topic                                |
| `statustext_pub` | `Publisher`    | Publisher for STATUSTEXT mode                                     |
| `debug_pub`      | `Publisher`    | Publisher for NAMED_VALUE_FLOAT mode                              |

---

## Configuration Parameters

| Parameter        | Type   | Default | Description                         |
| ---------------- | ------ | ------- | ----------------------------------- |
| `use_statustext` | `bool` | `true`  | Use STATUSTEXT format (recommended) |

---

## Key Functions and Why They Exist

| Function             | Why It Exists                                        | Variables Used                                   |
| -------------------- | ---------------------------------------------------- | ------------------------------------------------ |
| `__init__`           | Initialize ROS node, declare params, create sub/pubs | `use_statustext`, `geotag_sub`, `statustext_pub` |
| `geotag_callback`    | Convert incoming geotag to MAVLink and transmit      | `tx_count`, `use_statustext`                     |
| `_send_statustext`   | Format and publish STATUSTEXT message                | `statustext_pub`, `GEOTAG_PREFIX`                |
| `_send_named_values` | Alternative: send 3 NAMED_VALUE_FLOAT messages       | `debug_pub`                                      |

---

## Subscribers

| Topic                    | Type              | Callback          | Purpose                         |
| ------------------------ | ----------------- | ----------------- | ------------------------------- |
| `/drone1/disease_geotag` | `GeoPointStamped` | `geotag_callback` | Disease GPS from detection node |

## Publishers

| Topic                      | Type         | Purpose                              |
| -------------------------- | ------------ | ------------------------------------ |
| `/mavros/statustext/send`  | `StatusText` | STATUSTEXT mode transmission         |
| `/mavros/debug_value/send` | `DebugValue` | NAMED_VALUE_FLOAT mode (alternative) |

---

## Message Encoding

### STATUSTEXT Mode (Default)

```
Format: "GEOTAG:28.423000,77.524900,6.7"
         ^^^^^^ ^^^^^^^^^ ^^^^^^^^^ ^^^
         prefix latitude  longitude altitude

Max length: 50 characters (MAVLink limit)
Severity: 6 (INFO)
```

### NAMED_VALUE_FLOAT Mode (Alternative)

```
Message 1: name="d_lat", value=28.423000
Message 2: name="d_lon", value=77.524900
Message 3: name="d_alt", value=6.7
```

---

## Data Flow

```
detection_and_geotag_node
     │
     │ GeoPointStamped
     │ /drone1/disease_geotag
     ▼
┌─────────────────────────┐
│      telem_tx_node      │
│                         │
│  • Receive geotag       │
│  • Format: "GEOTAG:..." │
│  • Publish STATUSTEXT   │
└─────────────┬───────────┘
              │ StatusText
              │ /mavros/statustext/send
              ▼
         MAVROS
              │
              │ MAVLink STATUSTEXT
              ▼
      Cube Orange+ USB (SERIAL0)
              │
              │ (ArduPilot auto-forwards to SERIAL1)
              ▼
      Cube Orange+ TELEM1 (SERIAL1)
              │
              ▼
         Radio (AIR)
              │
         ~~~~ RF ~~~~
              │
              ▼
         GCS / Drone 2
```

---

## Manual Testing

### Start MAVROS (Hardware - USB)

```bash
ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200
```

### Start MAVROS (SITL)

```bash
ros2 launch mavros apm.launch fcu_url:=udp://:14551@127.0.0.1:14550
```

### Run telem_tx

```bash
cd ~/Documents/ROS-Arkairo/drone1_ws && source install/setup.zsh
ros2 run telem_tx telem_tx_node
```

### Send Test Geotag

```bash
ros2 topic pub /drone1/disease_geotag geographic_msgs/msg/GeoPointStamped \
  "{header: {frame_id: 'map'}, position: {latitude: 28.4223, longitude: 77.5249, altitude: 6.7}}" --once
```

### Expected Output

```
[INFO] [telem_tx_node]: TX #1: lat=28.423000, lon=77.524900, alt=6.7
```

### Monitor STATUSTEXT

```bash
ros2 topic echo /mavros/statustext/send
```

---

## Error Handling

| Scenario                | Behavior                             |
| ----------------------- | ------------------------------------ |
| MAVROS not running      | Messages published but not delivered |
| GeoPointStamped invalid | Transmitted as-is (no validation)    |
| Text > 50 chars         | Truncated to 50 chars                |

---

**Last Updated**: January 13, 2026
