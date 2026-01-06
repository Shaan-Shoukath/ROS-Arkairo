# Drone 2 Navigation Node - Developer Documentation

## Overview

**File**: `drone2_ws/src/drone2_navigation/drone2_navigation/drone2_navigation_node.py`  
**Package**: `drone2_navigation`  
**Node Name**: `drone2_navigation_node`  
**Author**: Shaan Shoukath

## Purpose

Receives GPS geotags from Drone-1 via telemetry and navigates to each location for spraying at 22 feet altitude. Features timeout-based RTL and dynamic target acceptance.

---

## Hardware Preconditions

```
Raspberry Pi 5 ─── UART/USB ───► Cube Orange+
Sprayer Relay ───► Cube AUX Output
Telemetry Radio ───► Cube TELEM1
GPS ───► Cube Orange+
```

## ArduPilot Parameters

```
SERIAL1_PROTOCOL = 2     # TELEM1 = MAVLink2 (radio)
SERIAL1_BAUD = 57        # 57600
SERIAL2_PROTOCOL = 2     # TELEM2 = MAVLink2 (Pi)
SERIAL2_BAUD = 921       # 921600
MAV_FORWARD = 1          # Forward messages
SYSID_THISMAV = 2        # Drone 2 system ID
RELAY_PIN = 54           # Sprayer relay (AUX5)
```

---

## State Machine

```
IDLE → WAIT_FCU → WAIT_TARGET → SET_GUIDED → ARM → TAKEOFF → WAIT_TAKEOFF
    → NAVIGATE → ARRIVED → WAIT_FOR_NEXT → (new target or RTL)
```

## Key Parameters

```yaml
# Altitude Settings
takeoff_altitude_m: 6.7     # 22 feet
navigation_altitude_m: 6.7  # 22 feet

# Navigation
arrival_radius_m: 3.0       # Consider "arrived"
wait_timeout_sec: 15.0      # Wait before RTL
setpoint_rate_hz: 10.0      # ≥10Hz required

# Start Location (Competition Setup)
use_gps_home: true          # true = GPS lock, false = use config below
start_latitude: 10.0478     # Custom start latitude
start_longitude: 76.3303    # Custom start longitude
start_altitude_m: 0.0       # Ground altitude (meters)
```

---

## Subscribers

| Topic                         | Type        | Purpose              |
| ----------------------------- | ----------- | -------------------- |
| `/drone2/target_position`     | NavSatFix   | Geotag from telem_rx |
| `/mavros/state`               | State       | FCU status           |
| `/mavros/local_position/pose` | PoseStamped | Current position     |

## Publishers

| Topic                             | Type        | Purpose                 |
| --------------------------------- | ----------- | ----------------------- |
| `/mavros/setpoint_position/local` | PoseStamped | Position commands       |
| `/drone2/arrival_status`          | Bool        | Triggers detection node |
| `/drone2/navigation_status`       | String      | Current state           |

---

## Launch Command

```bash
ros2 run drone2_navigation drone2_navigation_node --ros-args \
  --params-file src/drone2_navigation/config/navigation_params.yaml
```

---

**Last Updated**: January 5, 2026
