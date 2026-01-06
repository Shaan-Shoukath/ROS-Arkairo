# Drone-1 Hardware Testing Guide

> **Companion Computer**: Raspberry Pi 5  
> **Flight Controller**: Cube Orange+  
> **ROS Version**: ROS 2 Jazzy  
> **Author**: Shaan Shoukath

---

## 1. Hardware Connections

### Wiring Diagram

```
Cube Orange+ (TELEM2) ─────────→ Raspberry Pi 5 (GPIO UART)
     TX ──────────────────────→ RX (GPIO 15 / Pin 10)
     RX ──────────────────────→ TX (GPIO 14 / Pin 8)
     GND ─────────────────────→ GND (Pin 6)
     5V ──────────────────────→ 5V (Pin 2) [Optional]
```

### Serial Port Configuration

| Device      | Port                             | Baud Rate |
| ----------- | -------------------------------- | --------- |
| GPIO UART   | `/dev/ttyAMA0`                   | 921600    |
| USB Adapter | `/dev/ttyUSB0` or `/dev/ttyACM0` | 921600    |

---

## 2. Pre-Flight Setup

### 2.1 On Raspberry Pi 5

```bash
# Clone/update repository
cd ~/ROS-Arkairo
git pull origin main

# Build workspace
cd drone1_ws
source /opt/ros/jazzy/setup.zsh
colcon build

# Source workspace
source install/setup.zsh
```

### 2.2 Configure Cube Orange+ (Mission Planner)

Set these parameters permanently:

| Parameter          | Value  | Description                          |
| ------------------ | ------ | ------------------------------------ |
| `RTL_ALT`          | 0      | Maintain current altitude during RTL |
| `RTL_ALT_FINAL`    | 0      | Land immediately on arrival          |
| `SERIAL2_BAUD`     | 921600 | TELEM2 baud rate                     |
| `SERIAL2_PROTOCOL` | 2      | MAVLink2                             |

**Save to EEPROM**: Config → Full Parameter List → Write Params

### 2.3 Prepare Mission File

Place your KML file in:

```
~/ROS-Arkairo/drone1_ws/missions/
```

---

## 3. Terminal Setup (Real Hardware)

Open 3 terminals on the Raspberry Pi 5:

### Terminal 1: MAVROS

```bash
ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyAMA0:921600
```

Expected output:

```
[mavros] FCU: ArduCopter ...
[mavros] FCU connected
```

⏳ **Wait for "FCU connected" before proceeding**

---

### Terminal 2: KML Lane Planner

```bash
cd ~/ROS-Arkairo/drone1_ws
source install/setup.zsh
ros2 run kml_lane_planner kml_lane_planner_node --ros-args -p require_gps_home:=true
```

Expected output:

```
[kml_lane_planner_node] Home position set: (10.0478, 76.3303)
[kml_lane_planner_node] Processing KML: /path/to/mission.kml
[kml_lane_planner_node] Generated N waypoints
[kml_lane_planner_node] Waypoints logged to: mission_waypoints_YYYYMMDD.waypoints
```

---

### Terminal 3: Navigation Node

```bash
cd ~/ROS-Arkairo/drone1_ws
source install/setup.zsh
ros2 run drone1_navigation drone1_navigation_node
```

Expected output:

```
[drone1_navigation_node] Drone-1 Flight Controller - Position Control Mode
[drone1_navigation_node] Takeoff: 6.7m | Navigate: 6.7m
[drone1_navigation_node] Mission received: N waypoints from M lanes
[drone1_navigation_node] FSM: IDLE → WAIT_FOR_FCU
[drone1_navigation_node] FCU connected
[drone1_navigation_node] FSM: WAIT_FOR_FCU → WAIT_FOR_MISSION
[drone1_navigation_node] FSM: WAIT_FOR_MISSION → SET_GUIDED
[drone1_navigation_node] Setting RTL_ALT=0 (maintain current altitude)
[drone1_navigation_node] ✅ RTL_ALT parameter set successfully
```

---

## 4. Flight Sequence

The navigation node automatically executes:

```
IDLE → WAIT_FOR_FCU → WAIT_FOR_MISSION → SET_GUIDED → ARM → TAKEOFF → NAVIGATE → RTL
```

| State            | Action                                 |
| ---------------- | -------------------------------------- |
| WAIT_FOR_FCU     | Wait for Cube Orange+ connection       |
| WAIT_FOR_MISSION | Wait for waypoints from KML planner    |
| SET_GUIDED       | Set GUIDED mode + configure RTL params |
| ARM              | Stream setpoints, then arm motors      |
| TAKEOFF          | Climb to 6.7m (22 feet)                |
| NAVIGATE         | Follow survey waypoints                |
| RTL              | Return to launch at current altitude   |

---

## 5. Configuration Files

All configurable parameters are in YAML files:

### Navigation: `drone1_ws/src/drone1_navigation/config/navigation_params.yaml`

```yaml
takeoff_altitude_m: 6.7 # 22 feet
navigation_altitude_m: 6.7 # Survey altitude
waypoint_arrival_radius_m: 3.0
```

### KML Planner: `drone1_ws/src/kml_lane_planner/config/planner_params.yaml`

```yaml
altitude_m: 6.7 # Waypoint altitude
lane_spacing_m: 15.0 # Distance between survey lines
default_home_lat: 10.0478
default_home_lon: 76.3303
```

---

## 6. Pre-Flight Checklist

### Hardware

- [ ] Cube Orange+ powered and connected via serial
- [ ] GPS has solid lock (green LED)
- [ ] Battery connected and fully charged
- [ ] Props installed correctly
- [ ] RC transmitter on and bound

### Software

- [ ] All 3 terminals running (MAVROS, KML Planner, Navigation)
- [ ] "FCU connected" in MAVROS terminal
- [ ] Waypoints generated in KML Planner terminal
- [ ] RTL_ALT=0 confirmed in Navigation terminal

### ArduPilot Parameters

- [ ] RTL_ALT = 0
- [ ] RTL_ALT_FINAL = 0
- [ ] Parameters saved to EEPROM

---

## 7. Troubleshooting

### MAVROS won't connect

```bash
# Check serial port permissions
sudo chmod 666 /dev/ttyAMA0

# Or add user to dialout group
sudo usermod -a -G dialout $USER
# Then logout and login again
```

### No waypoints generated

```bash
# Check if KML file exists
ls ~/ROS-Arkairo/drone1_ws/missions/*.kml

# Check GPS lock
ros2 topic echo /mavros/global_position/global --once
```

### Drone won't arm

Check MAVLink console for prearm failures:

- GPS not locked
- RC not connected
- Accelerometer calibration needed

---

## 8. Emergency Procedures

| Issue                 | Action                         |
| --------------------- | ------------------------------ |
| Loss of control       | Switch to STABILIZE mode on RC |
| Navigation node crash | Drone enters RTL automatically |
| FCU disconnection     | Drone enters RTL automatically |
| Low battery           | Drone enters RTL automatically |

**RC Override**: Switch to STABILIZE or LOITER mode at any time to regain manual control.

---

## 9. Quick Reference

### Start Commands

```bash
# Terminal 1: MAVROS
ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyAMA0:921600

# Terminal 2: KML Planner
cd ~/ROS-Arkairo/drone1_ws && source install/setup.zsh
ros2 run kml_lane_planner kml_lane_planner_node --ros-args -p require_gps_home:=true

# Terminal 3: Navigation
cd ~/ROS-Arkairo/drone1_ws && source install/setup.zsh
ros2 run drone1_navigation drone1_navigation_node
```

### Monitor Topics

```bash
ros2 topic echo /mavros/state                    # FCU connection
ros2 topic echo /mavros/global_position/global   # GPS position
ros2 topic echo /drone1/navigation_status        # Flight state
ros2 topic echo /mission/lane_segments           # Waypoints
```
