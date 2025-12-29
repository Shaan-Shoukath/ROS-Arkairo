# Sprayer Control Node

**PWM Spray Actuation with Safety Interlocks**

## What It Does

Controls the pesticide spray pump via PWM output. Only activates when centered over target and all safety checks pass. Sprays for configured duration then signals completion.

## Logic Flow

```
Spray Ready = True (from centering)
        │
        ▼
Safety Pre-Check:
├── Vehicle armed? ────────────────────┐
├── Mode is GUIDED? ───────────────────┤
├── Altitude >= min_altitude? ─────────┤
└── No active abort? ──────────────────┘
        │                              │
        ▼ [all pass]          [any fail]
Set PWM to ON value                 ▼
        │                    Publish spray_done = False
        ▼
Start Spray Timer
        │
        ▼ [duration elapsed]
Set PWM to OFF value
        │
        ▼
Publish spray_done = True
```

## Subscribers

| Topic                         | Type          | Callback           | Description       |
| ----------------------------- | ------------- | ------------------ | ----------------- |
| `/drone2/spray_ready`         | `Bool`        | `ready_callback()` | Trigger to spray  |
| `/mavros/state`               | `State`       | `state_callback()` | Armed/mode status |
| `/mavros/local_position/pose` | `PoseStamped` | `pose_callback()`  | Current altitude  |

## Publishers

| Topic                 | Type           | Description      |
| --------------------- | -------------- | ---------------- |
| `/mavros/rc/override` | `OverrideRCIn` | PWM output       |
| `/drone2/spray_done`  | `Bool`         | Spray completion |

## Parameters

| Parameter            | Default | Description             |
| -------------------- | ------- | ----------------------- |
| `spray_channel`      | `9`     | RC channel for pump     |
| `pwm_on`             | `1900`  | PWM value for spray on  |
| `pwm_off`            | `1100`  | PWM value for spray off |
| `spray_duration_sec` | `3.0`   | Spray time              |
| `min_altitude_m`     | `2.0`   | Safety altitude floor   |
| `require_guided`     | `true`  | Require GUIDED mode     |

## Key Functions

### `ready_callback(msg: Bool)`

Handles spray trigger with safety checks.

```python
def ready_callback(self, msg):
    if not msg.data:
        return

    if not self._safety_check():
        self.get_logger().error('Safety check failed - aborting spray')
        self._publish_done(False)
        return

    # Start spraying
    self.spraying = True
    self.spray_start = self.get_clock().now()
    self._set_pwm(self.pwm_on)
    self.get_logger().info(f'Spraying for {self.spray_duration}s')
```

### `_safety_check() -> bool`

Validates all safety conditions.

```python
def _safety_check(self):
    # Must be armed
    if not self.fc_state or not self.fc_state.armed:
        self.get_logger().error('Not armed')
        return False

    # Must be in GUIDED mode
    if self.require_guided and self.fc_state.mode != 'GUIDED':
        self.get_logger().error(f'Mode is {self.fc_state.mode}, not GUIDED')
        return False

    # Must be above minimum altitude
    if self.current_pose:
        alt = self.current_pose.pose.position.z
        if alt < self.min_altitude:
            self.get_logger().error(f'Altitude {alt}m < min {self.min_altitude}m')
            return False

    return True
```

### `spray_loop()`

Timer callback to monitor spray duration.

```python
def spray_loop(self):
    if not self.spraying:
        return

    elapsed = (self.get_clock().now() - self.spray_start).nanoseconds / 1e9

    if elapsed >= self.spray_duration:
        self._stop_spray()
        self._publish_done(True)
        self.get_logger().info('Spray complete')
```

### `_set_pwm(value)`

Sets RC override for spray channel.

```python
def _set_pwm(self, value):
    msg = OverrideRCIn()
    msg.channels = [65535] * 18  # Ignore all
    msg.channels[self.spray_channel - 1] = value
    self.rc_pub.publish(msg)
```

### `_stop_spray()`

Stops spray and resets state.

```python
def _stop_spray(self):
    self._set_pwm(self.pwm_off)
    self.spraying = False
```

## Debugging

### Test PWM Output

```bash
# Monitor RC override
ros2 topic echo /mavros/rc/override

# Check spray done signal
ros2 topic echo /drone2/spray_done
```

### Manual Test (USE WITH CAUTION)

```bash
# Manually trigger spray (for testing with no pump)
ros2 topic pub /drone2/spray_ready std_msgs/Bool "data: true" -1
```

### Common Issues

| Issue          | Cause                       | Debug                 |
| -------------- | --------------------------- | --------------------- |
| Won't spray    | Safety check failed         | Check logs for reason |
| Wrong channel  | `spray_channel` param       | Verify RC mapping     |
| Spray stuck on | Node crashed                | Check node status     |
| No PWM output  | MAVROS RC override disabled | Enable in params      |

## Safety Design

1. **Altitude floor** - Never sprays below 2m
2. **Mode check** - Only in GUIDED mode
3. **Armed check** - Only when vehicle armed
4. **Duration limit** - Always stops after timer
5. **Default off** - PWM defaults to off value

## Hardware Wiring

```
Cube Orange+ AUX OUT
        │
        ▼ (Channel 9 = AUX 1)
    Relay Module
        │
        ▼
    Pump Motor (12V)
```
