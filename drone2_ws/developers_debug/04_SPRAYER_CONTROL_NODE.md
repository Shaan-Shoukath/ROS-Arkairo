# Sprayer Control Node - Developer Documentation

## Overview

**File**: `drone2_ws/src/sprayer_control/sprayer_control/sprayer_control_node.py`  
**Package**: `sprayer_control`  
**Node Name**: `sprayer_control_node`  
**Purpose**: Controls spray pump actuation with safety checks, priming delay, and hardware/sim modes

## What This Node Does

Safe spray pump controller with priming:

1. **Waits for spray_ready signal** from detection/centering
2. **Performs safety checks** (armed, mode)
3. **Priming delay** (2.5s default) - allows mechanism to stabilize
4. **Activates spray pump** via PWM topic (sim) or MAVROS servo (hardware)
5. **Times spray duration**
6. **Deactivates pump** and signals completion

## State Machine

```
       IDLE
        ↓ spray_ready=True + safety pass
     PRIMING (2.5s delay)
        ↓
     SPRAYING (5s duration)
        ↓
   Set output OFF
        ↓
  Publish spray_done
        ↓
       IDLE
```

## Modes: FC Relay vs Simulation

### FC Relay Mode (`control_mode: mavros` - default)

- Sends `MAV_CMD_DO_SET_RELAY` command via MAVROS to Orange Cube+
- Direct control of ArduPilot relay output
- **Recommended for real hardware**

**ArduPilot Configuration**:

```
RELAY_PIN = 54           # AUX5 pin (or 50-57 for AUX1-8)
RELAY_DEFAULT = 0        # Start OFF
```

**Wiring**:

```
Orange Cube+ AUX5 Pin
    ├── Signal → Relay Signal IN
    ├── +5V → Relay VCC
    └── GND → Relay GND
```

### Simulation Mode (`control_mode: pwm`)

- Publishes PWM value to `/drone2/pwm_spray` topic
- For SITL testing and development
- Automatically enabled when `use_sim: true`

### GPIO Mode (`control_mode: gpio`)

- Direct Pi 5 GPIO pin control (alternative)
- Requires `gpiozero` library on Pi

## Parameters

| Parameter               | Default | Description                                     |
| ----------------------- | ------- | ----------------------------------------------- |
| **Control Mode**        |         |                                                 |
| `control_mode`          | mavros  | `mavros` (FC), `gpio` (Pi 5), or `pwm` (sim)    |
| `use_sim`               | true    | Auto-switches to PWM mode for SITL              |
| **MAVROS/FC Settings**  |         |                                                 |
| `relay_channel`         | 0       | Relay instance (0 = Relay1)                     |
| `use_relay_command`     | true    | Use DO_SET_RELAY (true) or DO_SET_SERVO (false) |
| `servo_channel`         | 9       | AUX servo channel (if use_relay_command=false)  |
| **Timing**              |         |                                                 |
| `spray_start_delay_sec` | 2.5     | Priming delay before pump activates             |
| `spray_duration_sec`    | 5.0     | How long to spray                               |
| **PWM Settings**        |         |                                                 |
| `pwm_off`               | 1000    | Pulse width for OFF (µs)                        |
| `pwm_on`                | 2000    | Pulse width for ON (µs)                         |
| **Safety**              |         |                                                 |
| `require_armed`         | true    | Must be armed to spray                          |
| `require_guided`        | true    | Must be in GUIDED mode                          |
| `min_altitude_m`        | 2.0     | Minimum safe altitude                           |
| **GPIO Settings**       |         |                                                 |
| `gpio_pin`              | 17      | BCM pin (if control_mode=gpio)                  |
| `gpio_active_high`      | true    | Relay polarity                                  |

## Subscribers

| Topic                 | Type              | Purpose                  |
| --------------------- | ----------------- | ------------------------ |
| `/drone2/spray_ready` | std_msgs/Bool     | Spray activation trigger |
| `/mavros/state`       | mavros_msgs/State | Flight controller state  |

## Publishers

| Topic                | Type            | Purpose                   |
| -------------------- | --------------- | ------------------------- |
| `/drone2/spray_done` | std_msgs/Bool   | Spray complete signal     |
| `/drone2/pwm_spray`  | std_msgs/UInt16 | PWM value (sim mode only) |

## Service Clients

| Service               | Type                    | Purpose                       |
| --------------------- | ----------------------- | ----------------------------- |
| `/mavros/cmd/command` | mavros_msgs/CommandLong | Servo control (hardware mode) |

## Safety Features

1. **Require Armed**: Won't spray if motors disarmed
2. **Require GUIDED**: Won't spray in AUTO or other modes
3. **State Check**: Ignores requests during PRIMING or SPRAYING
4. **Auto Shutoff**: Timer ensures spray stops after duration
5. **Startup OFF**: Spray output OFF on node start
6. **Shutdown OFF**: Spray output OFF on node shutdown

## Testing

### SITL Testing (Simulation Mode)

```bash
# Terminal 1: Start node with safety disabled
ros2 run sprayer_control sprayer_control_node --ros-args \
  -p require_armed:=false -p require_guided:=false

# Terminal 2: Trigger spray
ros2 topic pub --once /drone2/spray_ready std_msgs/msg/Bool "data: true"

# Terminal 3: Monitor
ros2 topic echo /drone2/pwm_spray
ros2 topic echo /drone2/spray_done
```

**Expected Output**:

1. Log: "Priming delay started (2.5s)..."
2. Wait 2.5 seconds
3. Log: "Spraying started"
4. PWM goes to 2000 on /drone2/pwm_spray
5. Wait 5 seconds
6. Log: "Spraying complete (priming: 2.5s + spray: 5.0s = 7.5s total)"
7. PWM goes to 1000
8. spray_done = True published

### Hardware Testing

```bash
# Launch with hardware mode
ros2 launch drone2_bringup drone2_sprayer.launch.py use_sim:=false

# Verify servo command in Mission Planner or via mavros topics
```

## Timing Diagram

```
Time:    0s        2.5s                    7.5s
         |---------|------------------------|
State:   PRIMING      SPRAYING              IDLE
Output:  OFF          ON                    OFF
Signal:  spray_ready  (internal)            spray_done
```

## Common Issues

| Issue                      | Cause               | Solution                         |
| -------------------------- | ------------------- | -------------------------------- |
| Spray not activating       | Safety check failed | Check armed/GUIDED state         |
| No priming delay           | Old code version    | Verify spray_start_delay_sec     |
| Hardware relay not working | Wrong servo channel | Check SERVO10_FUNCTION=0         |
| MAVROS service timeout     | MAVROS not running  | Start MAVROS before sprayer node |

## SITL Simulation Testing

### Test Without Hardware (Bypass Safety Checks)

```bash
# Terminal 1: Run node with safety checks disabled
ros2 run sprayer_control sprayer_control_node --ros-args \
  -p require_armed:=false \
  -p require_guided:=false \
  -p min_altitude_m:=0.0

# Terminal 2: Trigger spray
ros2 topic pub --once /drone2/spray_ready std_msgs/msg/Bool "data: true"

# Terminal 3: Monitor PWM and completion
ros2 topic echo /drone2/pwm_spray
ros2 topic echo /drone2/spray_done
```

**Expected Sequence**:

1. PWM → 2000 (ON)
2. Wait 5 seconds (default duration)
3. PWM → 1000 (OFF)
4. spray_done → True

### Full SITL Test (With Safety Checks)

```bash
# Terminal 1: ArduPilot SITL + arm + takeoff
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map -l 10.0478,76.3303,0,0
# Then: arm throttle, mode guided, takeoff 10

# Terminal 2: MAVROS
ros2 launch mavros apm.launch.py fcu_url:=udp://:14550@127.0.0.1:14555

# Terminal 3: Sprayer node (with safety enabled)
ros2 run sprayer_control sprayer_control_node

# Terminal 4: Trigger spray (will only work if armed + guided + altitude)
ros2 topic pub --once /drone2/spray_ready std_msgs/msg/Bool "data: true"
```

---

**Last Updated**: January 1, 2026  
**Maintainer**: Shaan Shoukath  
**Safety Critical**: Review all parameters before field deployment!
