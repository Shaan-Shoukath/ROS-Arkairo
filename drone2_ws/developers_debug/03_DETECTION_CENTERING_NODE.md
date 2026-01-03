# Detection & Centering Node - Developer Documentation

## Overview

**File**: `drone2_ws/src/local_detection_and_centering/local_detection_and_centering/detection_centering_node.py`  
**Package**: `local_detection_and_centering`  
**Node Name**: `detection_centering_node`  
**Author**: Shaan Shoukath

## Purpose

Complete spray cycle after arrival at geotag:

1. **DETECTING** - Finds disease using HSV color detection
2. **CENTERING** - Visual servoing to align drone over target
3. **DESCENDING** - Lowers to spray altitude (3m)
4. **SPRAYING** - Activates sprayer for set duration (3s)
5. **ASCENDING** - Returns to navigation altitude (6.7m)
6. **COMPLETED** - Ready for next geotag

## State Machine

```
IDLE → DETECTING → CENTERING → DESCENDING → SPRAYING → ASCENDING → COMPLETED → IDLE
  ↑        ↓            ↓           ↓                                    │
  └── (timeout) ───────┴────────────┘                                    │
  └──────────────────────────────────────────────────────────────────────┘
```

## Key Parameters

Located in: `config/detection_centering_params.yaml`

```yaml
# >>> ALTITUDE SETTINGS (EASY TO CHANGE) <<<
navigation_altitude_m: 6.7 # 22 feet - survey/transit height
spray_altitude_m: 3.0 # Spray height
descent_velocity_mps: 0.3 # Descent speed
ascend_velocity_mps: 0.5 # Ascend speed
spray_duration_sec: 3.0 # Spray time at each target

# Camera (720p square)
image_width: 720
image_height: 720
```

## Subscribers

| Topic                         | Type        | Purpose               |
| ----------------------------- | ----------- | --------------------- |
| `/drone2/arrival_status`      | Bool        | Trigger from nav node |
| `/camera/image_raw`           | Image       | Video feed            |
| `/mavros/local_position/pose` | PoseStamped | Current altitude      |

## Publishers

| Topic                               | Type         | Purpose            |
| ----------------------------------- | ------------ | ------------------ |
| `/mavros/setpoint_velocity/cmd_vel` | TwistStamped | XYZ velocity       |
| `/drone2/spray_ready`               | Bool         | Sprayer activation |
| `/drone2/detection_status`          | Bool         | Detection result   |

## SITL Testing

### T1: SITL

```bash
cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter --console --map -l 10.0478,76.3303,0,0 -w
```

### T2: MAVROS

```bash
ros2 launch mavros apm.launch.py fcu_url:=udp://:14550@127.0.0.1:14555
```

### T3: Navigation Node

```bash
cd ~/Documents/ROSArkairo/drone2_ws && source install/setup.zsh
ros2 run drone2_navigation drone2_navigation_node --ros-args --params-file src/drone2_navigation/config/navigation_params.yaml
```

### T4: Detection Node

```bash
cd ~/Documents/ROSArkairo/drone2_ws && source install/setup.zsh
ros2 run local_detection_and_centering detection_centering_node --ros-args --params-file src/local_detection_and_centering/config/detection_centering_params.yaml
```

### T5: Send Target

```bash
ros2 topic pub /drone2/target_position sensor_msgs/msg/NavSatFix "{latitude: 10.0481, longitude: 76.3306, altitude: 10.0}" --once
```

### T6: Trigger Detection (after arrival)

```bash
ros2 topic pub /drone2/arrival_status std_msgs/msg/Bool "{data: true}" --once
```

## Expected Log Output

```
State: IDLE → DETECTING
Disease detected after 3 attempts
State: DETECTING → CENTERING
Target centered - descending to spray altitude
State: CENTERING → DESCENDING
[DESCENDING] Current: 6.5m → Target: 3.0m
Reached spray altitude: 3.1m - starting spray
State: DESCENDING → SPRAYING
[SPRAYING] 3s remaining...
Spray complete - ascending to navigation altitude
State: SPRAYING → ASCENDING
[ASCENDING] Current: 3.2m → Target: 6.7m
Reached navigation altitude: 6.6m - ready for next target
State: ASCENDING → COMPLETED
```

---

**Last Updated**: January 2, 2026
