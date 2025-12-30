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

## Modes: Simulation vs Hardware

### Simulation Mode (`use_sim: true` - default)
- Publishes PWM value to `/drone2/pwm_spray` topic
- For SITL testing and development
- Launch: `ros2 launch drone2_bringup drone2_sprayer.launch.py use_sim:=true`

### Hardware Mode (`use_sim: false`)
- Sends `MAV_CMD_DO_SET_SERVO` command via MAVROS
- Direct control of ArduPilot servo output (relay)
- Launch: `ros2 launch drone2_bringup drone2_sprayer.launch.py use_sim:=false`

**ArduPilot Configuration for Hardware Mode**:
```
SERVO10_FUNCTION = 0     # Passthrough (ROS control)
SERVO10_MIN = 1000       # PWM minimum
SERVO10_MAX = 2000       # PWM maximum
SERVO10_TRIM = 1000      # Default OFF
```

## Parameters

| Parameter                | Default | Description                                    |
| ------------------------ | ------- | ---------------------------------------------- |
| `use_sim`                | true    | Simulation mode (PWM topic vs MAVROS servo)    |
| `spray_start_delay_sec`  | 2.5     | Priming delay before pump activates            |
| `spray_duration_sec`     | 5.0     | How long to spray                              |
| `servo_channel`          | 10      | ArduPilot servo channel (hardware mode)        |
| `pwm_channel`            | 9       | PWM channel for sim mode                       |
| `pwm_off`                | 1000    | Pulse width for OFF (µs)                       |
| `pwm_on`                 | 2000    | Pulse width for ON (µs)                        |
| `require_armed`          | true    | Must be armed to spray                         |
| `require_guided`         | true    | Must be in GUIDED mode                         |
| `min_altitude_m`         | 2.0     | Minimum safe altitude                          |

## Subscribers

| Topic                   | Type             | Purpose                        |
| ----------------------- | ---------------- | ------------------------------ |
| `/drone2/spray_ready`   | std_msgs/Bool    | Spray activation trigger       |
| `/mavros/state`         | mavros_msgs/State| Flight controller state        |

## Publishers

| Topic                   | Type             | Purpose                        |
| ----------------------- | ---------------- | ------------------------------ |
| `/drone2/spray_done`    | std_msgs/Bool    | Spray complete signal          |
| `/drone2/pwm_spray`     | std_msgs/UInt16  | PWM value (sim mode only)      |

## Service Clients

| Service                 | Type                    | Purpose                        |
| ----------------------- | ----------------------- | ------------------------------ |
| `/mavros/cmd/command`   | mavros_msgs/CommandLong | Servo control (hardware mode)  |

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

| Issue                      | Cause                  | Solution                         |
| -------------------------- | ---------------------- | -------------------------------- |
| Spray not activating       | Safety check failed    | Check armed/GUIDED state         |
| No priming delay           | Old code version       | Verify spray_start_delay_sec     |
| Hardware relay not working | Wrong servo channel    | Check SERVO10_FUNCTION=0         |
| MAVROS service timeout     | MAVROS not running     | Start MAVROS before sprayer node |

---

**Last Updated**: December 31, 2024  
**Safety Critical**: Review all parameters before field deployment!
