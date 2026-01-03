# Telem TX Node - Developer Documentation

## Overview

**File**: `drone1_ws/src/telem_tx/telem_tx/telem_tx_node.py`  
**Package**: `telem_tx`  
**Node Name**: `telem_tx_node`  
**Author**: Shaan Shoukath

## Purpose

Transmits disease geotag data from Drone-1 to Drone-2 via MAVLink telemetry radio.

## Key Parameters

```yaml
serial_port: "/dev/ttyUSB0" # Telemetry radio port
baud_rate: 57600 # MAVLink baud rate
```

## Subscribers

| Topic                    | Type            | Purpose                   |
| ------------------------ | --------------- | ------------------------- |
| `/drone1/disease_geotag` | GeoPointStamped | Detection GPS to transmit |

## Publishers

| Topic                  | Type   | Purpose             |
| ---------------------- | ------ | ------------------- |
| `/drone1/telem_status` | String | Transmission status |

## Data Flow

```
Detection Node → /drone1/disease_geotag → telem_tx → MAVLink Radio → Drone-2
```

---

**Last Updated**: January 2, 2026
