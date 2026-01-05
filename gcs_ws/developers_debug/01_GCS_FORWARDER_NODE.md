# GCS Forwarder Node - Developer Documentation

## Overview

**File**: `gcs_ws/src/gcs_forwarder/gcs_forwarder/forwarder_node.py`  
**Package**: `gcs_forwarder`  
**Author**: Shaan Shoukath

## Purpose

Event-driven geotag relay from Drone 1 to Drone 2 via GCS. Uses pymavlink to listen on one telemetry radio and forward geotag messages to the other.

---

## Hardware Preconditions

```
USB Port 0 (/dev/ttyUSB0) ◄─── Drone 1 Radio (Pair A - Ground)
USB Port 1 (/dev/ttyUSB1) ◄─── Drone 2 Radio (Pair B - Ground)
```

## Dependencies

```bash
pip install pymavlink
```

---

## Message Filtering

**Only forwards:**

- STATUSTEXT messages with prefix `GEOTAG:`
- Origin SYSID = 1 (Drone 1)

**Does NOT forward:**

- Monitoring telemetry
- Drone 2 messages
- Duplicate geotags

---

## Data Flow

```
Drone 1 → Radio A ~~~ RF ~~~ Radio A (Ground)
    → GCS Forwarder (filters + logs)
    → Radio B (Ground) ~~~ RF ~~~ Radio B → Drone 2
```

---

## Configuration

```yaml
drone1_port: "/dev/ttyUSB0"
drone2_port: "/dev/ttyUSB1"
baud_rate: 57600
source_sysid: 1
geotag_prefix: "GEOTAG:"
```

---

## Launch Command

```bash
cd ~/Documents/ROSArkairo/gcs_ws
python3 src/gcs_forwarder/gcs_forwarder/forwarder_node.py --drone1 /dev/ttyUSB0 --drone2 /dev/ttyUSB1
```

---

**Last Updated**: January 5, 2026
