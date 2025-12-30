# Sprayer Control Node - Developer Documentation

## Overview

**File**: `drone2_ws/src/sprayer_control/sprayer_control/sprayer_control_node.py`  
**Package**: `sprayer_control`  
**Node Name**: `sprayer_control_node`  
**Purpose**: Controls spray pump actuation with safety checks

## What This Node Does

Safe spray pump controller:

1. **Waits for spray_ready signal** from detection/centering
2. **Performs safety checks** (armed, altitude, mode)
3. **Activates spray pump** via PWM output
4. **Times spray duration**
5. **Deactivates pump** and signals completion

## Core Logic

```python
State: IDLE
      ↓
spray_ready = True received
      ↓
Safety checks passed?
      ↓ YES
Set PWM to ON (2000µs)
      ↓
Start timer
      ↓
Wait spray_duration seconds
      ↓
Set PWM to OFF (1000µs)
      ↓
Publish spray_done = True
      ↓
Return to IDLE
```

**Safety-First Design**: Multiple checks before allowing spray activation.

## Safety Checks

### 1. Flight Controller State

```python
if require_armed and not fc_state.armed:
    REJECT("Drone not armed")

if require_guided and fc_state.mode != "GUIDED":
    REJECT("Not in GUIDED mode")
```

**Why?** Prevents spray on ground or during wrong flight mode.

### 2. Altitude Check

```python
if current_altitude < min_altitude:
    REJECT("Too low - might spray ground crew")
```

**Purpose**: Ensures drone is airborne before spraying.

### 3. Already Spraying Check

```python
if currently_spraying:
    REJECT("Spray already active")
```

**Purpose**: Prevents overlapping spray cycles.

## PWM Control

### What is PWM?

**PWM (Pulse Width Modulation)**: Control method using pulse timing

**Servo/ESC Standard**:

- **1000µs**: Minimum (OFF)
- **1500µs**: Center
- **2000µs**: Maximum (ON)
- **Frequency**: 50Hz (20ms period)

### Spray Pump Control

```python
PWM OFF = 1000µs  →  Pump relay OFF
PWM ON  = 2000µs  →  Pump relay ON
```

**Hardware Setup**:

```
Pixhawk AUX OUT (e.g., AUX1/CH9)
        ↓
    PWM Signal
        ↓
   Relay Module
        ↓
   12V Pump
```

**ArduPilot Configuration**:

```
SERVOx_FUNCTION = 0 (disabled, manual control)
Where x = PWM channel (e.g., SERVO9 for AUX1)
```

## Subscribers

### 1. `/drone2/spray_ready` (std_msgs/Bool)

- **Source**: Detection & Centering Node
- **Purpose**: Trigger spray activation
- **QoS**: RELIABLE (guaranteed delivery)
- **Logic**: `True` = start spray, `False` = ignored

### 2. `/mavros/state` (mavros_msgs/State)

- **Source**: MAVROS
- **Purpose**: Flight controller status for safety checks
- **Fields Used**: `armed`, `mode`

## Publishers

### 1. `/drone2/spray_done` (std_msgs/Bool)

- **Trigger**: After spray duration complete
- **Purpose**: Signals navigation node for next target
- **Value**: `True` = success, `False` = failure

### 2. `/drone2/pwm_spray` (std_msgs/UInt16)

- **Rate**: On state change (OFF → ON → OFF)
- **Purpose**: PWM value for hardware interface
- **Values**: 1000 (OFF) or 2000 (ON)

**Note**: This topic would connect to a hardware PWM driver node (not shown in current system).

## Parameters

| Parameter            | Default | Description                       |
| -------------------- | ------- | --------------------------------- |
| `pwm_channel`        | 9       | ArduPilot SERVO number (AUX1 = 9) |
| `pwm_off`            | 1000    | Pulse width for OFF (µs)          |
| `pwm_on`             | 2000    | Pulse width for ON (µs)           |
| `spray_duration_sec` | 5.0     | How long to spray                 |
| `require_armed`      | True    | Must be armed to spray            |
| `require_guided`     | True    | Must be in GUIDED mode            |
| `min_altitude_m`     | 2.0     | Minimum safe altitude             |

## Key Functions

### `ready_callback()` - Spray Trigger Handler

```python
def ready_callback(self, msg: Bool):
    if not msg.data:
        return  # Only respond to True

    if self.spraying:
        log_warning("Already spraying")
        return

    if not self._safety_check():
        log_error("Safety check failed")
        self._publish_done(False)
        return

    # All checks passed - START SPRAYING
    self.spraying = True
    self.spray_start_time = self.get_clock().now()
    self._set_pwm(self.pwm_on)
    self.total_sprays += 1
```

### `_safety_check()` - Multi-Level Validation

```python
def _safety_check(self) -> bool:
    if self.fc_state is None:
        return False

    if self.require_armed and not self.fc_state.armed:
        log_error("Not armed")
        return False

    if self.require_guided and self.fc_state.mode != 'GUIDED':
        log_error("Not in GUIDED mode")
        return False

    if self.current_altitude < self.min_altitude:
        log_error(f"Altitude {self.current_altitude}m < {self.min_altitude}m")
        return False

    return True
```

**Built-in**: Boolean `and`, comparison operators

### `spray_loop()` - Duration Timer (10Hz)

```python
def spray_loop(self):
    if not self.spraying:
        return

    elapsed = (now() - spray_start_time).nanoseconds / 1e9

    if elapsed >= self.spray_duration:
        self._stop_spray()
        self._publish_done(True)
```

**Built-in**:

- `self.get_clock().now()` - ROS time
- `.nanoseconds` - Time in nanoseconds
- Division `/` and comparison `>=`

### `_set_pwm()` - PWM Output

```python
def _set_pwm(self, value: int):
    msg = UInt16()
    msg.data = value
    self.pwm_pub.publish(msg)
```

**Built-in**: Integer type

### `_stop_spray()` - Cleanup

```python
def _stop_spray(self):
    self.spraying = False
    self._set_pwm(self.pwm_off)
```

## Package Dependencies

### ROS2 Packages

- **rclpy**: Node, QoS, Timer
- **std_msgs**: Bool, UInt16
- **mavros_msgs**: State

### Python Standard Library

- **typing**: Optional type hint

## State Diagram

```
       IDLE
        ↓ spray_ready=True
    Safety Check
        ↓ PASS
    Set PWM ON
        ↓
    SPRAYING
        ↓ (timer)
    Duration elapsed?
        ↓ YES
    Set PWM OFF
        ↓
  Publish spray_done
        ↓
       IDLE
```

## Hardware Integration

### Required ArduPilot Setup

**1. Configure Servo Output**:

```
SERVO9_FUNCTION = 0      # Passthrough (manual control from ROS)
SERVO9_MIN = 1000        # PWM minimum
SERVO9_MAX = 2000        # PWM maximum
SERVO9_TRIM = 1000       # Default OFF
```

**2. RC Channel Override**:
Enable RC override on channel 9:

```
# In MAVROS parameters
rc_override_mask: 256  # Bit 8 = Channel 9
```

**3. Physical Wiring**:

```
Pixhawk AUX1 (Signal) → Relay Module (Signal)
Pixhawk +5V → Relay Module (VCC)  [if needed]
Pixhawk GND → Relay Module (GND)
Relay NO/NC → Pump Power +12V
Battery GND → Pump GND
```

### Alternative: Direct MAVROS PWM

Instead of `/drone2/pwm_spray`, can use MAVROS directly:

```python
from mavros_msgs.msg import OverrideRCIn

rc_msg = OverrideRCIn()
rc_msg.channels = [65535] * 18  # 65535 = no change
rc_msg.channels[8] = 2000  # Channel 9 (index 8) = ON
self.rc_pub.publish(rc_msg)
```

## Safety Features

1. **Require Armed**: Won't spray if motors disarmed
2. **Require GUIDED**: Won't spray in AUTO or other modes
3. **Minimum Altitude**: Won't spray below safe height
4. **Already Spraying Check**: Prevents double-activation
5. **Auto Shutoff**: Timer ensures spray doesn't stay on
6. **Startup OFF**: PWM set to OFF on node start

## Testing Checklist

- [ ] Node starts with PWM OFF
- [ ] Rejects spray_ready when not armed
- [ ] Rejects spray_ready when altitude too low
- [ ] Rejects spray_ready when not in GUIDED
- [ ] Activates spray when all checks pass
- [ ] Sprays for correct duration
- [ ] Deactivates spray after duration
- [ ] Publishes spray_done = True
- [ ] Tracks spray count correctly

## Testing Without Hardware

**Simulate spray cycle**:

```bash
# Terminal 1: Start node
ros2 run sprayer_control sprayer_control_node --ros-args \
  -p require_armed:=false -p require_guided:=false -p min_altitude_m:=0.0

# Terminal 2: Trigger spray
ros2 topic pub --once /drone2/spray_ready std_msgs/msg/Bool "data: true"

# Terminal 3: Monitor
ros2 topic echo /drone2/pwm_spray
ros2 topic echo /drone2/spray_done
```

**Expected**:

1. PWM goes to 2000
2. Waits 5 seconds
3. PWM goes to 1000
4. spray_done = True published

## Common Issues

**Issue**: Spray not activating  
**Solution**: Check safety parameters, verify FC state

**Issue**: Spray stays on  
**Solution**: Emergency: Disarm drone. Check timer logic.

**Issue**: PWM not reaching hardware  
**Solution**: Verify SERVO function, check RC override enabled

**Issue**: Sprays too long/short  
**Solution**: Adjust `spray_duration_sec` parameter

## Calibration

**Spray Duration**:

1. Measure pump flow rate (mL/sec)
2. Calculate volume needed per spot
3. Set duration: `duration = volume / flow_rate`
4. Test and adjust based on coverage

**Example**:

- Flow rate: 100 mL/s
- Target volume: 500 mL per spot
- Duration: 500/100 = 5 seconds

---

**Last Updated**: December 30, 2025  
**Safety Critical**: Review all parameters before field deployment!
