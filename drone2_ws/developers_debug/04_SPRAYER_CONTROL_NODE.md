# Sprayer Control Node - Developer Documentation

## Overview

**File**: `drone2_ws/src/sprayer_control/sprayer_control/sprayer_control_node.py`  
**Package**: `sprayer_control`  
**Node Name**: `sprayer_control_node`  
**Author**: Shaan Shoukath

## Purpose

Controls sprayer pump/relay when triggered by detection node. Supports both simulation (PWM topic) and hardware (MAVROS servo/relay).

---

## Hardware Preconditions

```
Relay Module ───► Cube Orange+ AUX5 (GPIO 54)
Pump ───► Relay NO/COM terminals
12V Battery ───► Relay power
```

## ArduPilot Parameters

```
RELAY_PIN = 54           # AUX5 GPIO
RELAY_DEFAULT = 0        # OFF at startup
BRD_SAFETYENABLE = 0     # Disable safety for testing
```

---

## Key Parameters

```yaml
use_sim: true # PWM topic vs hardware
pwm_channel: 9
pwm_off: 1000
pwm_on: 2000
spray_duration_sec: 5.0
servo_channel: 10
```

---

## Subscribers

| Topic                 | Type | Purpose                |
| --------------------- | ---- | ---------------------- |
| `/drone2/spray_ready` | Bool | Trigger from detection |

## Publishers

| Topic                 | Type         | Purpose        |
| --------------------- | ------------ | -------------- |
| `/mavros/rc/override` | OverrideRCIn | PWM (sim)      |
| MAVROS servo command  | -            | Hardware relay |

---

## Pump Test

```bash
ros2 run sprayer_control pump_test --ros-args \
  -p relay_num:=0 \
  -p on_time_sec:=3.0
```

---

**Last Updated**: January 5, 2026
