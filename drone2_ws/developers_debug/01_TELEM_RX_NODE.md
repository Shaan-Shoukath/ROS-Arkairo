# Telem RX Node - Developer Documentation

## Overview

**File**: `drone2_ws/src/telem_rx/telem_rx/telem_rx_node.py`  
**Package**: `telem_rx`  
**Node Name**: `telem_rx_node`  
**Author**: Shaan Shoukath

## Purpose

Receives disease geotag telemetry from Drone-1 via MAVLink and republishes as ROS2 NavSatFix messages for the navigation node.

## Key Parameters

```yaml
serial_port: "/dev/ttyUSB0" # Telemetry radio port
baud_rate: 57600 # MAVLink baud rate
```

## Subscribers

None (receives via MAVLink serial)

## Publishers

| Topic                     | Type      | Purpose               |
| ------------------------- | --------- | --------------------- |
| `/drone2/target_position` | NavSatFix | Geotag for navigation |
| `/drone2/telem_status`    | String    | Connection status     |

## Data Flow

```
Drone-1 (telem_tx) → MAVLink Radio → telem_rx → /drone2/target_position → Navigation Node
```

---

**Last Updated**: January 2, 2026
