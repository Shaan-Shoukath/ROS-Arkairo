# Sprayer Control Node - Developer Documentation

## Overview

**File**: `drone2_ws/src/sprayer_control/sprayer_control/sprayer_control_node.py`  
**Package**: `sprayer_control`  
**Node Name**: `sprayer_control_node`  
**Author**: Shaan Shoukath

## Purpose

Controls the spray mechanism when triggered by the detection/centering node. Activates pump/relay when spray_ready signal received.

## Key Parameters

Located in: `config/sprayer_params.yaml`

```yaml
use_sim: true # true = PWM topic, false = MAVROS servo
servo_channel: 10 # ArduPilot servo channel
spray_pwm: 1900 # PWM value when spraying
idle_pwm: 1100 # PWM value when idle
```

## Subscribers

| Topic                 | Type | Purpose                     |
| --------------------- | ---- | --------------------------- |
| `/drone2/spray_ready` | Bool | Trigger from detection node |

## Publishers

| Topic                 | Type         | Purpose                    |
| --------------------- | ------------ | -------------------------- |
| `/mavros/rc/override` | OverrideRCIn | PWM commands (sim mode)    |
| MAVROS servo command  | -            | Hardware servo (real mode) |

## Integration Flow

```
Detection Node → spray_ready=true → Sprayer Node → Activate pump
                                                 → Wait spray_duration
Detection Node → spray_ready=false → Sprayer Node → Deactivate pump
```

---

**Last Updated**: January 2, 2026
