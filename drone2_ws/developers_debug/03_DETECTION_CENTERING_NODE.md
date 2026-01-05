# Detection & Centering Node - Developer Documentation

## Overview

**File**: `drone2_ws/src/local_detection_and_centering/local_detection_and_centering/detection_centering_node.py`  
**Package**: `local_detection_and_centering`  
**Node Name**: `detection_centering_node`  
**Author**: Shaan Shoukath

## Purpose

Complete spray cycle after arrival at geotag:
DETECTING → CENTERING → DESCENDING → SPRAYING → ASCENDING → COMPLETED

---

## Hardware Preconditions

```
Pi Camera 3 ─── CSI ───► Raspberry Pi 5
Sprayer connected to Cube AUX output
```

## ArduPilot Parameters

```
RELAY_PIN = 54           # Sprayer relay (AUX5)
BRD_SAFETYENABLE = 0     # Disable for ground testing
```

---

## State Machine

```
IDLE → DETECTING → CENTERING → DESCENDING → SPRAYING → ASCENDING → COMPLETED
  ↑        ↓            ↓           ↓                                    │
  └── (timeout) ───────┴────────────┘                                    │
  └──────────────────────────────────────────────────────────────────────┘
```

## Key Parameters

```yaml
use_sim: true # Skip detection in SITL
navigation_altitude_m: 6.7 # 22 feet
spray_altitude_m: 3.0 # Spray height
descent_velocity_mps: 0.3
ascend_velocity_mps: 0.5
spray_duration_sec: 3.0
```

---

## Subscribers

| Topic                         | Type        | Purpose          |
| ----------------------------- | ----------- | ---------------- |
| `/drone2/arrival_status`      | Bool        | Trigger from nav |
| `/camera/image_raw`           | Image       | Video feed       |
| `/mavros/local_position/pose` | PoseStamped | Altitude         |

## Publishers

| Topic                               | Type         | Purpose                |
| ----------------------------------- | ------------ | ---------------------- |
| `/mavros/setpoint_velocity/cmd_vel` | TwistStamped | XYZ velocity           |
| `/drone2/spray_ready`               | Bool         | Sprayer activation     |
| `/mavros/statustext/send`           | StatusText   | Mission Planner status |

---

## Visual Servoing (Centering)

```
Pixel Error = target_center - image_center
Velocity = PID(error)
Centered when error < 30 pixels
```

---

## Launch Command

```bash
ros2 run local_detection_and_centering detection_centering_node --ros-args \
  --params-file src/local_detection_and_centering/config/detection_centering_params.yaml
```

---

**Last Updated**: January 5, 2026
