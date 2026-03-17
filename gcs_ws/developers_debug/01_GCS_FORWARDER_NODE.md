# GCS Forwarder Node - Developer Documentation

## Overview

| Property        | Value                                                      |
| --------------- | ---------------------------------------------------------- |
| **File**        | `gcs_ws/src/gcs_forwarder/gcs_forwarder/forwarder_node.py` |
| **Package**     | `gcs_forwarder`                                            |
| **Node Name**   | `forwarder_node` (standalone Python, not ROS)              |
| **Config File** | Command-line arguments                                     |
| **Maintainer**  | Shaan Shoukath                                             |

---

## Purpose

Event-driven geotag relay from Drone 1 to Drone 2 via GCS laptop. Uses **pymavlink** to listen on one telemetry radio and forward geotag STATUSTEXT messages to the other radio.

---

## Hardware Setup

```
┌─────────────────────────────────────────────────────────────┐
│                       GCS LAPTOP                            │
│                                                             │
│   ┌─────────────────┐                                       │
│   │ forwarder_node  │                                       │
│   │   (Python)      │                                       │
│   │                 │                                       │
│   │  Listens on:    │                                       │
│   │  /dev/ttyUSB0   │ ← Radio A ground (from Drone 1)       │
│   │                 │                                       │
│   │  Forwards to:   │                                       │
│   │  /dev/ttyUSB1   │ → Radio B ground (to Drone 2)         │
│   └────────┬────────┘                                       │
│            │                                                │
│   ┌────────┴───────────────────────────────────┐            │
│   │                                            │            │
│   ▼                                            ▼            │
│ USB0 (/dev/ttyUSB0)                     USB1 (/dev/ttyUSB1) │
│   │                                            │            │
│   ▼                                            ▼            │
│ Radio A (Ground)                        Radio B (Ground)    │
│ Net ID: 25                              Net ID: 25          │
└─────┬───────────────────────────────────────────┬───────────┘
      │                                           │
 ~~~~ RF ~~~~                                ~~~~ RF ~~~~
      │                                           │
      ▼                                           ▼
Radio A (Air)                             Radio B (Air)
on Drone 1                                on Drone 2
```

### Radio Configuration (All 4 Radios)

| Setting    | Value                          |
| ---------- | ------------------------------ |
| **Net ID** | **25** (MUST be same on ALL 4) |
| Baud Rate  | 57600                          |
| Air Speed  | 64                             |
| TX Power   | 20 dBm                         |
| ECC        | Enabled                        |
| Mavlink    | Enabled                        |

---

## State Variables

| Variable        | Type                 | Purpose                                     |
| --------------- | -------------------- | ------------------------------------------- |
| `drone1_conn`   | `mavutil.connection` | MAVLink connection to Drone 1 radio         |
| `drone2_conn`   | `mavutil.connection` | MAVLink connection to Drone 2 radio         |
| `source_sysid`  | `int`                | Expected SYSID of Drone 1 (default: 1)      |
| `last_geotag`   | `str`                | Last forwarded geotag (duplicate detection) |
| `forward_count` | `int`                | Total geotags forwarded                     |
| `GEOTAG_PREFIX` | `str`                | Constant: `"GEOTAG:"`                       |

---

## Command-Line Arguments

| Argument   | Default        | Description               |
| ---------- | -------------- | ------------------------- |
| `--drone1` | `/dev/ttyUSB0` | Drone 1 radio USB port    |
| `--drone2` | `/dev/ttyUSB1` | Drone 2 radio USB port    |
| `--baud`   | `57600`        | Baud rate for both radios |
| `--sysid`  | `1`            | Expected source SYSID     |

---

## Key Functions and Why They Exist

| Function         | Why It Exists                                    | Variables Used                 |
| ---------------- | ------------------------------------------------ | ------------------------------ |
| `__init__`       | Initialize MAVLink connections to both radios    | `drone1_conn`, `drone2_conn`   |
| `run`            | Main loop - listen for messages, filter, forward | `drone1_conn`, `last_geotag`   |
| `forward_geotag` | Send geotag to Drone 2 radio                     | `drone2_conn`, `forward_count` |
| `shutdown`       | Clean disconnect and print stats                 | `forward_count`                |

---

## Message Filtering Logic

```python
# 1. Only STATUSTEXT messages
if msg.get_type() != 'STATUSTEXT':
    continue

# 2. Only from SYSID 1 (Drone 1)
if msg._header.srcSystem != self.source_sysid:
    continue

# 3. Only messages starting with "GEOTAG:"
if not text.startswith("GEOTAG:"):
    continue

# 4. Skip duplicates
if text == self.last_geotag:
    continue

# ✅ All checks passed - forward to Drone 2
self.forward_geotag(msg)
```

### What Gets Forwarded

- ✅ STATUSTEXT with `GEOTAG:` prefix from SYSID=1

### What Gets Filtered Out

- ❌ Heartbeat, GPS, attitude, etc.
- ❌ Messages from SYSID=2 (Drone 2)
- ❌ Non-GEOTAG STATUSTEXT
- ❌ Duplicate geotags

---

## Data Flow

```
Drone 1 (SYSID=1)
     │
     │ STATUSTEXT "GEOTAG:28.4223,77.5249,6.7"
     ▼
Radio A (Air) ~~~~~~~ RF ~~~~~~~ Radio A (Ground)
                                      │
                                      │ USB /dev/ttyUSB0
                                      ▼
                              ┌───────────────┐
                              │ GCS Forwarder │
                              │               │
                              │ • Filter      │
                              │   SYSID=1 ✓   │
                              │   GEOTAG: ✓   │
                              │               │
                              │ • Log         │
                              │               │
                              │ • Forward     │
                              └───────┬───────┘
                                      │ USB /dev/ttyUSB1
                                      ▼
Radio B (Ground) ~~~~~~~ RF ~~~~~~~ Radio B (Air)
                                      │
                                      ▼
                               Drone 2 (SYSID=2)
```

---

## Dependencies

```bash
pip install pymavlink
```

---

## Manual Testing

### Start Forwarder

```bash
cd ~/Documents/ROS-Arkairo/gcs_ws
python3 src/gcs_forwarder/gcs_forwarder/forwarder_node.py \
  --drone1 /dev/ttyUSB0 \
  --drone2 /dev/ttyUSB1 \
  --baud 57600
```

### Expected Startup Output

```
============================================================
GCS Geotag Forwarder
============================================================
Drone 1 port: /dev/ttyUSB0
Drone 2 port: /dev/ttyUSB1
Baud rate: 57600
Expected source SYSID: 1
============================================================
Connecting to Drone 1 radio on /dev/ttyUSB0...
  ✓ Connected
Connecting to Drone 2 radio on /dev/ttyUSB1...
  ✓ Connected

Forwarder ready. Listening for geotags...
Press Ctrl+C to stop.
```

### Expected Forwarding Output

```
[14:32:15] FORWARDED #1: GEOTAG:28.423000,77.524900,6.7
           Lat: 28.423000, Lon: 77.524900, Alt: 6.7m
[14:33:22] FORWARDED #2: GEOTAG:28.423100,77.525000,6.7
           Lat: 28.423100, Lon: 77.525000, Alt: 6.7m
```

---

## Why Use GCS Forwarder Instead of Just Radio?

Even though all radios have the same Net ID and can hear each other:

| Feature                 | GCS Forwarder                 | Radio-Only       |
| ----------------------- | ----------------------------- | ---------------- |
| **SYSID filtering**     | ✅ Only forwards from Drone 1 | ❌ All messages  |
| **Logging**             | ✅ Timestamped log            | ❌ None          |
| **Duplicate detection** | ✅ Prevents re-sending        | ❌ May duplicate |
| **Reduced RF traffic**  | ✅ Only geotags               | ❌ All telemetry |

---

## Troubleshooting

| Issue                    | Solution                                            |
| ------------------------ | --------------------------------------------------- |
| `/dev/ttyUSB0 not found` | Check radio USB connection, run `ls /dev/ttyUSB*`   |
| `Permission denied`      | Run `sudo usermod -a -G dialout $USER` and re-login |
| No messages received     | Check Net ID matches, verify Drone 1 is sending     |
| Messages not forwarding  | Check SYSID matches (default expects SYSID=1)       |

---

**Last Updated**: January 13, 2026
