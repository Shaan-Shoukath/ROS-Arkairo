# Sprayer Control Node - Developer Documentation

## Overview

| Property        | Value                                                                   |
| --------------- | ----------------------------------------------------------------------- |
| **File**        | `drone2_ws/src/sprayer_control/sprayer_control/sprayer_control_node.py` |
| **Package**     | `sprayer_control`                                                       |
| **Node Name**   | `sprayer_control_node`                                                  |
| **Config File** | `sprayer_control/config/sprayer_params.yaml`                            |
| **Maintainer**  | Shaan Shoukath                                                          |

## Purpose

Controls sprayer pump/relay when triggered by detection node. Supports simulation (PWM topic), GPIO (Pi 5), and MAVROS relay.

---

## State Variables

| Variable                | Type         | Purpose                        |
| ----------------------- | ------------ | ------------------------------ |
| `self.state`            | SprayState   | IDLE, PRIMING, or SPRAYING     |
| `self.control_mode`     | str          | "pwm", "gpio", or "mavros"     |
| `self.is_armed`         | bool         | FCU armed state (safety check) |
| `self.current_mode`     | str          | Flight mode (safety check)     |
| `self.state_start_time` | Time         | When current state entered     |
| `self.gpio_device`      | OutputDevice | gpiozero device (if mode=gpio) |
| `self.spray_active`     | bool         | Whether spray is currently on  |

---

## Configuration YAML → Code Mapping

```yaml
# sprayer_params.yaml                     # Where Used
control_mode: "pwm"               →  self.control_mode (determines output method)
pwm_channel: 9                    →  RC override channel index
pwm_off: 1000                     →  PWM value when off
pwm_on: 2000                      →  PWM value when on
spray_duration_sec: 5.0           →  How long to stay in SPRAYING state
spray_start_delay_sec: 2.5        →  Priming delay before pump activates
servo_channel: 10                 →  ArduPilot servo channel (MAVROS mode)
gpio_pin: 17                      →  Pi 5 GPIO pin (GPIO mode)
require_armed: true               →  Safety: only spray if armed
require_guided: true              →  Safety: only spray in GUIDED mode
min_altitude_m: 2.0               →  Safety: minimum altitude to spray
```

---

## Key Functions and Why They Exist

### Callbacks (React to external events)

| Function         | Why It Exists                           | Variables Used                  |
| ---------------- | --------------------------------------- | ------------------------------- |
| `ready_callback` | Centering node signals "ready to spray" | `state` → PRIMING if ready=True |
| `state_callback` | Monitor FCU state for safety checks     | `is_armed`, `current_mode`      |

### State Handlers (Called by spray_loop)

| Function     | Why It Exists             | Variables Used              |
| ------------ | ------------------------- | --------------------------- |
| `spray_loop` | Main state machine (10Hz) | `state`, `state_start_time` |

### Output Functions

| Function            | Why It Exists                         | Variables Used              |
| ------------------- | ------------------------------------- | --------------------------- |
| `_set_spray_output` | Route to correct control method       | `control_mode`              |
| `_set_pwm_topic`    | Publish PWM override (SITL)           | `pwm_channel`, `pwm_on/off` |
| `_set_gpio`         | Control Pi 5 GPIO directly            | `gpio_device`               |
| `_set_mavros_relay` | Send DO_SET_RELAY command             | `servo_channel`             |
| `_publish_done`     | Signal navigation that spray complete | spray_done=True             |

### Safety Functions

| Function        | Why It Exists                             | Variables Used                       |
| --------------- | ----------------------------------------- | ------------------------------------ |
| `_safety_check` | Verify armed/guided/altitude before spray | `is_armed`, `current_mode`, altitude |
| `_stop_spray`   | Emergency stop                            | Sets output off, state→IDLE          |
| `destroy_node`  | Ensure spray off on shutdown              | GPIO cleanup                         |

---

## State Machine

```
IDLE → (ready=true) → PRIMING (2.5s delay) → SPRAYING (5s) → spray_done → IDLE
                            ↓                     ↓
                      (safety fail)          (safety fail)
                            ↓                     ↓
                          IDLE ←─────────────────┘
```

### Why PRIMING exists

The pump needs time to pressurize. Activating immediately would spray air/weak stream.

---

## Control Modes

| Mode   | `control_mode` | How It Works                       | When To Use  |
| ------ | -------------- | ---------------------------------- | ------------ |
| PWM    | `pwm`          | Publishes to `/mavros/rc/override` | SITL testing |
| GPIO   | `gpio`         | Pi 5 GPIO via gpiozero             | Direct relay |
| MAVROS | `mavros`       | DO_SET_RELAY command               | Cube relay   |

---

## Safety Checks

Before spraying, the node verifies:

1. `require_armed`: Drone must be armed
2. `require_guided`: Must be in GUIDED mode
3. `min_altitude_m`: Must be above 2m

If any check fails, spray is aborted and spray_done=False is published.

---

## Subscribers

| Topic                 | Type  | Callback         | Purpose |
| --------------------- | ----- | ---------------- | ------- |
| `/drone2/spray_ready` | Bool  | `ready_callback` | Trigger |
| `/mavros/state`       | State | `state_callback` | Safety  |

## Publishers

| Topic                 | Type         | Purpose              |
| --------------------- | ------------ | -------------------- |
| `/drone2/spray_done`  | Bool         | Signal to navigation |
| `/mavros/rc/override` | OverrideRCIn | PWM control          |

---

## Launch Command

```bash
# SITL mode (PWM topic)
ros2 run sprayer_control sprayer_control_node --ros-args -p control_mode:=pwm

# Hardware mode (MAVROS relay)
ros2 run sprayer_control sprayer_control_node --ros-args \
  --params-file ~/Documents/ROS-Arkairo/drone2_ws/src/sprayer_control/config/sprayer_params.yaml \
  -p control_mode:=mavros
```

---

## Manual Testing

```bash
# Trigger spray manually
ros2 topic pub /drone2/spray_ready std_msgs/msg/Bool "{data: true}" --once
```

---

**Last Updated**: January 10, 2026
