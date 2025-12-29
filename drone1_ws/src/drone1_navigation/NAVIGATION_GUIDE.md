# Drone-1 Navigation Node Documentation

## Cube Orange+ with ArduPilot + MAVROS + ROS 2 Jazzy

---

## 🎯 Design Principles

This navigation node is built for **real hardware** (Cube Orange+ flight controller) with the following requirements:

1. **Explicit finite state machine** - No ambiguous state transitions
2. **Continuous setpoint publishing** - Mandatory for ArduPilot GUIDED mode
3. **No throttle control** - Companion computer never controls throttle
4. **No RC override** - Never hijacks RC channels
5. **Deterministic behavior** - Identical operation on SITL and real hardware
6. **Safe failure modes** - Always fails to HOLD or RTL

---

## 📡 Topic Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Raspberry Pi 5 (Companion)                       │
│                                                                     │
│  ┌───────────────────────────────────────────────────────────┐    │
│  │         Drone-1 Navigation Node (ROS 2 Jazzy)             │    │
│  │                                                             │    │
│  │  FSM: INIT → WAIT_FOR_FCU → SET_GUIDED → ARM              │    │
│  │       → TAKEOFF → NAVIGATE → HOLD → RTL                    │    │
│  └───────────────────────────────────────────────────────────┘    │
│                          ▲          │                              │
│                          │          │                              │
│         ┌────────────────┘          └──────────────┐              │
│         │  /mavros/state                            │              │
│         │  (FCU status)               /mavros/setpoint_position/   │
│         │                              local (10Hz continuous)     │
│         │  /mavros/local_position/                  │              │
│         │  pose                                     │              │
│         │                                           │              │
└─────────┼───────────────────────────────────────────┼──────────────┘
          │                                           │
          │            USB/Serial Connection           │
          │                                           │
┌─────────┼───────────────────────────────────────────┼──────────────┐
│         │                                           ▼              │
│  ┌──────────────────────────────────────────────────────────┐    │
│  │              Cube Orange+ (ArduPilot)                     │    │
│  │                                                            │    │
│  │  • Receives continuous position setpoints (≥10Hz)         │    │
│  │  • Interprets setpoints in GUIDED mode                    │    │
│  │  • Controls motors/ESCs via PWM                           │    │
│  │  • Handles all low-level flight control                   │    │
│  │  • Manages failsafes (battery, RC loss, etc.)            │    │
│  └──────────────────────────────────────────────────────────┘    │
│                          │                                        │
│                          ▼                                        │
│                      4× ESCs + Motors                             │
│                    (Direct PWM control)                           │
│                                                                   │
│              Cube Orange+ Flight Controller                       │
└───────────────────────────────────────────────────────────────────┘
```

---

## 🚫 Why the Companion NEVER Controls Throttle

### The ArduPilot Architecture

ArduPilot uses a **position controller hierarchy**:

```
High Level:     Waypoint/Position Commands (from companion)
                         ↓
Mid Level:      Position Controller (runs on Cube)
                         ↓
                Velocity Controller (runs on Cube)
                         ↓
Low Level:      Attitude Controller (runs on Cube)
                         ↓
                Motor Mixing (runs on Cube)
                         ↓
Hardware:       PWM Signals → ESCs → Motors
```

**The companion computer operates at the HIGH LEVEL only.**

When we publish to `/mavros/setpoint_position/local`, we're saying:

> "Cube, please navigate to position (x, y, z)"

The Cube's onboard controllers then:

1. Calculate the velocity needed to reach that position
2. Calculate the attitude (pitch/roll) needed for that velocity
3. Calculate the motor outputs needed for that attitude
4. Send PWM signals to the ESCs

**We never touch throttle, motor commands, or PWM signals.**

### Why This Is Better

1. **Safety**: Cube handles all low-level stabilization (400Hz+ rate)
2. **Reliability**: Cube has hardware failsafes (battery, RC loss)
3. **Simplicity**: We don't need to implement PID loops
4. **Hardware agnostic**: Same code works on any ArduPilot vehicle

### What Happens If We Try to Control Throttle?

If we used `/mavros/rc/override` to control throttle directly:

- ❌ We bypass the position controller
- ❌ We bypass safety checks
- ❌ We fight with the Cube's stabilization
- ❌ We lose failsafe protection
- ❌ **This is dangerous and explicitly forbidden**

---

## 📤 Why Continuous Setpoints Are Mandatory

### ArduPilot GUIDED Mode Behavior

When in GUIDED mode, ArduPilot expects:

- **Continuous position setpoints at ≥10Hz**
- If setpoints stop arriving, the Cube enters **"last known command" mode**

This means:

- The Cube will attempt to hold the last received position
- After a timeout (~3 seconds), it may trigger a failsafe

### Our Implementation

```python
# Setpoint timer runs at 10Hz (separate from FSM)
self.setpoint_timer = self.create_timer(
    1.0 / self.setpoint_rate,  # 10Hz
    self.publish_setpoint
)
```

**This timer runs continuously** regardless of FSM state:

- During TAKEOFF: Publishes climb setpoint
- During NAVIGATE: Publishes waypoint setpoint
- During HOLD: Publishes current position
- During ERROR: Publishes current position (safe hover)

### What Happens If Setpoints Stop?

1. **Immediate effect**: Cube holds last commanded position
2. **After ~3 seconds**: Cube may trigger GCS failsafe
3. **Failsafe action**: Depends on ArduPilot configuration (usually RTL or LAND)

**This is why our setpoint timer is completely independent of the FSM.**

---

## 🔄 State Machine Detailed Behavior

### State: INIT

- **Duration**: <100ms
- **Actions**: None
- **Transitions**: Immediately to WAIT_FOR_FCU

### State: WAIT_FOR_FCU

- **Waiting for**: `/mavros/state` message with `connected == true`
- **Timeout**: 30 seconds → ERROR
- **Failure reasons**:
  - MAVROS not running
  - USB/serial connection failed
  - Cube not powered
- **Transitions**: Connected → SET_GUIDED

### State: SET_GUIDED

- **Goal**: Switch flight mode to GUIDED
- **Service call**: `/mavros/set_mode` with `custom_mode = "GUIDED"`
- **Confirmation**: Wait for `/mavros/state.mode == "GUIDED"`
- **Failure reasons**:
  - Pre-arm checks failed (GPS, compass, etc.)
  - Radio safety switch not disengaged
  - Cube in a non-compatible mode
- **Transitions**: GUIDED confirmed → ARM

### State: ARM

- **Goal**: Arm the motors
- **Service call**: `/mavros/cmd/arming` with `value = true`
- **Confirmation**: Wait for `/mavros/state.armed == true`
- **Timeout**: 30 seconds → ERROR
- **Failure reasons**:
  - Pre-arm check failures (GPS, compass, accelerometer, etc.)
  - Battery voltage too low
  - Safety switch not disengaged
  - Not in GUIDED mode
- **Transitions**: Armed → TAKEOFF

### State: TAKEOFF

- **Goal**: Climb to takeoff altitude
- **Service call**: `/mavros/cmd/takeoff` with `altitude = takeoff_altitude_m`
- **Setpoints**: Published continuously (hold XY, climb Z)
- **Completion**: Current altitude ≥ (takeoff_altitude - 0.5m)
- **Timeout**: 60 seconds → ERROR
- **Failure reasons**:
  - Motors disarmed mid-flight
  - Lost GUIDED mode
  - Cube hit altitude limit
- **Transitions**: Altitude reached → NAVIGATE

### State: NAVIGATE

- **Goal**: Navigate to target waypoint
- **Setpoints**: Published continuously (target position)
- **Waypoint source**: `/mission/waypoint` topic (optional)
- **Failure detection**:
  - FCU disconnected → HOLD
  - Motors disarmed → ERROR
  - Lost GUIDED mode → HOLD
- **Timeout**: 300 seconds → HOLD
- **Transitions**: Manual (can transition to RTL or HOLD)

### State: HOLD

- **Goal**: Hold current position
- **Setpoints**: Published continuously (current position)
- **Purpose**: Safe state for troubleshooting
- **Transitions**: Manual only (or RTL command)

### State: RTL

- **Goal**: Return to launch point
- **Service call**: `/mavros/set_mode` with `custom_mode = "RTL"`
- **Behavior**: Cube handles navigation autonomously
- **Setpoints**: Still published (but Cube ignores them in RTL)

### State: ERROR

- **Goal**: Terminal error state
- **Setpoints**: Published (hold position if possible)
- **Recovery**: Requires node restart
- **Logging**: Clear error reason in logs

---

## 🛠️ How Cube Orange+ Interprets GUIDED Commands

### Position Setpoints (`/mavros/setpoint_position/local`)

Message type: `geometry_msgs/PoseStamped`

The Cube receives this as:

```
Target X (meters, local frame, forward)
Target Y (meters, local frame, right)
Target Z (meters, local frame, up)
Target Yaw (radians)
```

The Cube's onboard controllers then:

1. **Position controller**: Calculate velocity to reach target
2. **Velocity controller**: Calculate attitude to achieve velocity
3. **Attitude controller**: Calculate motor outputs for attitude

**Frame**: `map` frame (ENU convention: East-North-Up)

### Takeoff Command (`/mavros/cmd/takeoff`)

Service type: `mavros_msgs/CommandTOL`

Parameters:

- `altitude`: Target altitude (meters, relative to home)
- `latitude`, `longitude`: Ignored if 0 (uses current position)
- `yaw`: Target yaw (radians)

The Cube:

1. Arms motors if not already armed (on some firmware versions)
2. Climbs vertically to target altitude
3. Maintains XY position during climb
4. Completes when altitude ± 0.5m reached

### Arming (`/mavros/cmd/arming`)

Service type: `mavros_msgs/CommandBool`

The Cube checks:

- ✅ Pre-arm checks pass (GPS, compass, accel, gyro, baro)
- ✅ Battery voltage OK
- ✅ Safety switch disengaged (if present)
- ✅ In a mode that allows arming (GUIDED, STABILIZE, etc.)
- ✅ No critical errors or warnings

If **any** check fails, arming is rejected with a status message.

### Mode Changes (`/mavros/set_mode`)

Service type: `mavros_msgs/SetMode`

For `custom_mode = "GUIDED"`:

- Cube enters autonomous mode
- Expects continuous position/velocity setpoints
- Companion computer has full control
- RC input still available for manual override

For `custom_mode = "RTL"`:

- Cube navigates to home position autonomously
- Climbs to RTL altitude
- Returns to launch point
- Descends and lands

---

## 🧪 Testing on SITL vs Real Hardware

### SITL (Software In The Loop)

```bash
# Start ArduCopter SITL
sim_vehicle.py -v ArduCopter -f quad --console --map

# Start MAVROS
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://:14550@127.0.0.1:14555

# Start navigation node
ros2 run drone1_navigation drone1_navigation_node
```

**Differences from real hardware**:

- ✅ Faster boot time
- ✅ No GPS delay
- ✅ Perfect sensors (no noise)
- ❌ Simplified physics
- ❌ No hardware failsafes

### Real Hardware (Cube Orange+)

```bash
# Connect via USB/serial
# Cube should be powered and connected to Pi via USB

# Start MAVROS
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=serial:///dev/ttyUSB0:921600

# Start navigation node
ros2 run drone1_navigation drone1_navigation_node
```

**Additional considerations**:

- ⏰ Wait for GPS fix (~30 seconds)
- 🔋 Check battery voltage
- 🧭 Calibrate compass outdoors
- 📡 Verify RC connection
- 🔴 Disengage safety switch

**The same navigation node runs on both** - no code changes needed.

---

## 📊 Parameter Reference

| Parameter                | Default | Unit    | Description                               |
| ------------------------ | ------- | ------- | ----------------------------------------- |
| `takeoff_altitude_m`     | 10.0    | meters  | Target altitude for takeoff               |
| `setpoint_rate_hz`       | 10.0    | Hz      | Setpoint publishing rate (≥10Hz required) |
| `fcu_timeout_sec`        | 30.0    | seconds | Timeout waiting for FCU connection        |
| `arming_timeout_sec`     | 30.0    | seconds | Timeout waiting for arming                |
| `takeoff_timeout_sec`    | 60.0    | seconds | Timeout for takeoff completion            |
| `navigation_timeout_sec` | 300.0   | seconds | Max time in navigation state              |

---

## 🚨 Common Issues & Solutions

### Issue: "FCU connection timeout"

- **Cause**: MAVROS not connected to Cube
- **Check**: USB cable connected, Cube powered, correct `/dev/ttyUSB*`
- **Fix**: Verify MAVROS `fcu_url` parameter

### Issue: "Arming rejected"

- **Cause**: Pre-arm checks failed
- **Check**: GPS fix, compass calibration, safety switch
- **Fix**: Check ArduPilot messages in QGroundControl/Mission Planner

### Issue: "Lost GUIDED mode"

- **Cause**: RC mode switch changed, or failsafe triggered
- **Fix**: Ensure RC is in correct mode, check battery voltage

### Issue: "Takeoff timeout"

- **Cause**: Cube not climbing
- **Check**: Motors spinning, battery voltage, altitude estimate
- **Fix**: Verify EKF is healthy, check `EK3_ALT_SOURCE`

### Issue: Drone drifts during navigation

- **Cause**: GPS accuracy, wind, or EKF issues
- **Fix**: Improve GPS fix (RTK), tune position controller, check vibration levels

---

## 📚 References

- [ArduPilot GUIDED mode documentation](https://ardupilot.org/copter/docs/ac2_guidedmode.html)
- [MAVROS API reference](http://wiki.ros.org/mavros)
- [Cube Orange+ documentation](https://docs.cubepilot.org/user-guides/autopilot/the-cube-module-overview)
- [ArduPilot MAVLink command list](https://mavlink.io/en/messages/common.html)

---

## ✅ Quality Checklist

- [x] No RC override usage
- [x] No throttle control
- [x] No blocking loops or `sleep()`
- [x] Explicit FSM with clear states
- [x] Continuous setpoint publishing (≥10Hz)
- [x] Proper timeout handling
- [x] Clear error logging
- [x] Safe failure modes (HOLD/RTL)
- [x] Identical behavior on SITL and real hardware
- [x] Survives FCU disconnects

---

**This navigation node is ready for flight review and real hardware testing.**
