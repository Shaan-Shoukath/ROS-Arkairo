# Multi-Drone Agricultural System

**Made by Shaan Shoukath**

A production-grade ROS2 dual-drone system for **fully autonomous** field surveying and precision spraying, coordinated through a Ground Control Station (GCS).

![ROS2](https://img.shields.io/badge/ROS2-Humble%20|%20Iron%20|%20Jazzy-blue)
![License](https://img.shields.io/badge/License-Apache%202.0-green)
![Packages](https://img.shields.io/badge/Packages-19-orange)
![Autonomous](https://img.shields.io/badge/Mode-Fully%20Autonomous-brightgreen)

---

## 🏗️ Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    FULLY AUTONOMOUS OPERATION                           │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  Drone-1 (Survey)          GCS              Drone-2 (Sprayer)          │
│  ┌───────────────┐    ┌──────────┐    ┌─────────────────────┐         │
│  │ KML→Auto-Arm  │    │ Target   │    │ IDLE until geotag  │         │
│  │ Takeoff       │    │ Receiver │    │ Auto-Arm on 1st    │         │
│  │ Survey        │───►│ Mission  │───►│ Navigate→Spray     │         │
│  │ Detect→Geotag │    │ Router   │    │ 5s Wait→Next/RTL   │         │
│  │ RTL           │    │          │    │                     │         │
│  └───────────────┘    └──────────┘    └─────────────────────┘         │
│                                                                         │
│  ═══════════════════════ TELEMETRY LINK ═══════════════════════════   │
│                                                                         │
│  🛡️ TF-Luna Object Avoidance: Handled by ArduPilot (Cube Orange+)      │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 📁 Workspace Structure

| Workspace   | Packages | Description                                          |
| ----------- | -------- | ---------------------------------------------------- |
| `drone1_ws` | 7        | Survey drone - KML planning, navigation, detection   |
| `gcs_ws`    | 4        | Ground Control Station - routing and coordination    |
| `drone2_ws` | 8        | Sprayer drone - navigation, centering, spray control |

---

## 📖 Node Documentation

### Drone-1 (Survey System)

| Node                   | Description                             | Documentation                                             |
| ---------------------- | --------------------------------------- | --------------------------------------------------------- |
| **KML Lane Planner**   | Converts KML boundaries to survey paths | [📄 README](drone1_ws/src/kml_lane_planner/README.md)     |
| **Drone-1 Navigation** | Executes survey waypoints via MAVROS    | [📄 README](drone1_ws/src/drone1_navigation/README.md)    |
| **Detection & Geotag** | Disease detection with GPS ray-casting  | [📄 README](drone1_ws/src/detection_and_geotag/README.md) |

### Ground Control Station

| Node                | Description                            | Documentation                                         |
| ------------------- | -------------------------------------- | ----------------------------------------------------- |
| **Target Receiver** | Validates and filters incoming targets | [📄 README](gcs_ws/src/gcs_target_receiver/README.md) |
| **Mission Router**  | Dispatches missions to Drone-2         | [📄 README](gcs_ws/src/gcs_mission_router/README.md)  |

### Drone-2 (Sprayer System)

| Node                     | Description                             | Documentation                                             |
| ------------------------ | --------------------------------------- | --------------------------------------------------------- |
| **GCS Downlink**         | Converts GCS commands to local triggers | [📄 README](drone2_ws/src/gcs_to_d2_downlink/README.md)   |
| **Drone-2 Navigation**   | Navigates to target locations           | [📄 README](drone2_ws/src/drone2_navigation/README.md)    |
| **Local Detection**      | Confirms disease presence on arrival    | [📄 README](drone2_ws/src/local_detection/README.md)      |
| **Centering Controller** | PID visual servoing for alignment       | [📄 README](drone2_ws/src/centering_controller/README.md) |
| **Sprayer Control**      | PWM actuation with safety checks        | [📄 README](drone2_ws/src/sprayer_control/README.md)      |
| **Mission Manager**      | State machine and GCS reporting         | [📄 README](drone2_ws/src/mission_manager/README.md)      |

---

## 🔍 What Each Node Does

### Drone-1 (Survey System)

#### 🗺️ KML Lane Planner Node

Watches the `missions/` folder for KML files defining field boundaries. When a new KML is detected, it **automatically triggers the full mission sequence**:

- Parses the KML polygon coordinates
- Converts WGS84 (GPS) to local ENU coordinates for path planning
- Generates parallel survey lines (lawnmower pattern)
- Optimizes starting waypoint closest to home position
- Publishes `LaneSegmentArray` messages to trigger navigation

**Key Features**: Smart corner detection, configurable lane spacing, automatic home position fallback for SITL

---

#### 🧭 Drone-1 Navigation Node

**FULLY AUTONOMOUS** - Executes survey mission with automatic arming, takeoff, and RTL:

- Subscribes to `LaneSegmentArray` for mission waypoints
- **Auto-arms** when mission is received
- **Auto-takes off** to mission altitude
- Navigates survey pattern via MAVROS `GlobalPositionTarget`
- **Auto-RTL** when survey complete
- State machine: `IDLE → ARMING → TAKING_OFF → NAVIGATING → MISSION_COMPLETE → RETURNING_HOME`

**Key Features**: Zero manual intervention required, configurable via parameters:

- `auto_arm`, `auto_takeoff`, `auto_rtl` (all default `true`)
- `takeoff_altitude_m` (default **6.7m / 22 feet**)

---

#### 📷 Image Capture Node

Camera abstraction layer that captures and publishes raw images during survey flight:

- Initializes USB camera via OpenCV
- Publishes `sensor_msgs/Image` at configurable frame rate
- Publishes `sensor_msgs/CameraInfo` with camera intrinsics
- Supports simulation mode with generated test images

**Key Features**: Configurable resolution, auto-retry on camera failure, test mode for SITL

---

#### 🔬 Detection and Geotag Node

Processes camera frames to detect crop disease (yellow spots) and compute GPS coordinates:

- Subscribes to camera images, GPS, IMU, and altitude data
- Performs HSV color thresholding to detect yellow disease patches
- Converts pixel coordinates to GPS using ray-casting (IMU orientation + camera intrinsics)
- De-duplicates detections within configurable radius
- Logs all detections to CSV file

**Key Features**: Configurable HSV thresholds, severity classification, duplicate filtering, debug visualization

---

#### 📡 D1 to GCS Uplink Node

Forwards validated disease geotags from Drone-1 to the Ground Control Station:

- Queues incoming geotags for reliable transmission
- Rate-limits transmissions to avoid network congestion
- Implements retry logic for failed transmissions
- Publishes heartbeat for link monitoring

**Key Features**: Configurable queue size, retry mechanism, transmission statistics

---

### Ground Control Station

#### 🎯 GCS Target Receiver Node

Receives and validates target reports from Drone-1:

- Validates GPS coordinates are within acceptable bounds
- Checks against configurable geofence
- Filters duplicate targets within time window and distance threshold
- Assigns unique target IDs
- Saves validated targets to JSON file

**Key Features**: Geofence validation, time-based deduplication, persistent logging

---

#### 🚦 GCS Mission Router Node

Controls mission dispatch to Drone-2 based on validated targets and drone status:

- Maintains queue of pending targets
- Monitors Drone-2 operational status (`IDLE`, `NAVIGATING`, `SPRAYING`, etc.)
- Only dispatches when Drone-2 is available and not on active mission
- Implements hourly rate limiting for safety
- Sends target geotag and mission start trigger

**Key Features**: Auto-dispatch mode, status monitoring, rate limiting, queue management

---

### Drone-2 (Sprayer System)

#### 📥 GCS to D2 Downlink Node

Bridges GCS commands to local Drone-2 mission execution:

- Receives target geotags from GCS telemetry link
- Validates coordinates and checks distance from current position
- Waits for mission start trigger before dispatching
- Converts GCS commands to local `NavSatFix` for navigation

**Key Features**: Coordinate validation, max distance check, timeout handling

---

#### 🛫 Drone-2 Navigation Node

Navigates to target location using global position setpoints:

- Subscribes to target coordinates from downlink
- Publishes `GlobalPositionTarget` via MAVROS
- Monitors distance to target using Haversine calculation
- Publishes arrival status when within configurable radius
- Implements navigation timeout for safety

**Key Features**: Continuous setpoint publishing, timeout protection, arrival detection

---

#### 🔎 Local Detection Node

Confirms disease presence upon arrival at target location:

- Activates when `arrival_status` is received
- Processes camera frames to re-detect disease locally
- Requires configurable number of consecutive detections for confirmation
- Publishes bounding box for centering controller

**Key Features**: Multi-frame confirmation, timeout protection, false positive filtering

---

#### 🎯 Centering Controller Node

Fine-tunes drone position to align spray nozzle over target using visual servoing:

- Uses PID control to minimize pixel error from image center
- Subscribes to bounding box from local detection
- Publishes velocity commands via MAVROS
- Stops and signals ready when target is centered (within pixel threshold)

**Key Features**: Configurable PID gains, velocity limiting, timeout protection

---

#### 💦 Sprayer Control Node

Controls spray actuation via PWM output with safety checks:

- Waits for `spray_ready` signal from centering controller
- Performs safety checks (armed status, flight mode, altitude)
- Actuates spray pump for configurable duration
- Publishes `spray_done` when complete

**Key Features**: PWM control, safety interlocks, spray duration timer, statistics tracking

---

#### 📊 Mission Manager Node

**FULLY AUTONOMOUS** - Manages complete mission lifecycle with smart sequencing:

- **Stays IDLE** until first geotag received (never arms without mission)
- **Auto-arms and takes off** on first geotag
- Tracks state: `IDLE → ARMING → TAKING_OFF → NAVIGATING → DETECTING → CENTERING → SPRAYING → WAITING_FOR_NEXT`
- **5-second wait window** after each spray for next geotag
- **Auto-RTL** if no geotag received within wait window
- Reports all state changes to GCS

**Key Features**: Configurable `wait_timeout_sec` (default 5s), MAVROS service integration, deterministic behavior

---

## 🚀 Quick Start

> **📖 For complete simulation and real flight instructions, see [DEPLOYMENT.md](DEPLOYMENT.md)**

### Prerequisites

```bash
# ROS2 dependencies
sudo apt install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras
sudo apt install ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-vision-msgs
sudo apt install ros-${ROS_DISTRO}-geographic-msgs

# Python dependencies
pip3 install scipy geopy
```

### Build All Workspaces

```bash
# Drone-1
cd ~/Documents/ROSArkairo/drone1_ws
colcon build --symlink-install
source install/setup.bash

# GCS
cd ~/Documents/ROSArkairo/gcs_ws
colcon build --symlink-install
source install/setup.bash

# Drone-2
cd ~/Documents/ROSArkairo/drone2_ws
colcon build --symlink-install
source install/setup.bash
```

### Launch Systems

```bash
# Terminal 1 - Drone-1 Survey
ros2 launch drone1_bringup drone1_survey.launch.py

# Terminal 2 - GCS
ros2 launch gcs_bringup gcs.launch.py

# Terminal 3 - Drone-2 Sprayer
ros2 launch drone2_bringup drone2_sprayer.launch.py
```

### Run a Mission (Fully Autonomous)

```bash
# Drop KML file to start - everything else is automatic!
cp field_boundary.kml ~/Documents/ROSArkairo/drone1_ws/missions/

# Drone-1 will: Arm → Takeoff (22ft) → Survey → Detect → Publish geotags → RTL
# Drone-2 will: Wait → Arm on geotag → Fly → Spray → Wait 5s → Next/RTL
```

---

## 📡 Topic Reference

### Drone-1 ↔ GCS

| Topic                | Direction | Type            | Description     |
| -------------------- | --------- | --------------- | --------------- |
| `/gcs/target_report` | D1 → GCS  | GeoPointStamped | Disease geotags |

### GCS ↔ Drone-2

| Topic                   | Direction | Type            | Description     |
| ----------------------- | --------- | --------------- | --------------- |
| `/drone2/target_geotag` | GCS → D2  | GeoPointStamped | Target location |
| `/drone2/mission_start` | GCS → D2  | Bool            | Mission trigger |
| `/drone2/status`        | D2 → GCS  | String          | Current state   |

---

## 🔧 Key Algorithms

| Algorithm              | Location             | Description                                |
| ---------------------- | -------------------- | ------------------------------------------ |
| **ENU↔GPS Conversion** | Lane Planner         | WGS84 to local metric coordinates          |
| **Ray-Casting GPS**    | Detection Node       | Pixel to GPS using camera intrinsics + IMU |
| **PID Control**        | Centering Controller | Visual servoing for target alignment       |
| **Haversine Distance** | All navigation       | GPS distance calculation                   |

---

## ⚠️ Non-Negotiable Rules

| Rule                                 | Reason                         |
| ------------------------------------ | ------------------------------ |
| ❌ ROS never touches TF-Luna         | Flight controller owns sensors |
| ❌ GCS never sends low-level control | Separation of concerns         |
| ❌ No drone-to-drone direct links    | GCS coordination only          |
| ✅ MAVROS is the only FC interface   | Standardized MAVLink           |
| ✅ One node = one responsibility     | Clean architecture             |

---

## 📦 Package Summary

| Workspace | Message Pkgs | Node Pkgs | Launch Pkgs |
| --------- | ------------ | --------- | ----------- |
| drone1_ws | 1            | 5         | 1           |
| gcs_ws    | 1            | 2         | 1           |
| drone2_ws | 1            | 6         | 1           |
| **Total** | 3            | 13        | 3           |

---

## 🧪 SITL Testing Guide (Drone-1)

### Prerequisites

- ArduPilot SITL installed (`sim_vehicle.py` available)
- MAVROS installed for ROS 2
- All workspaces built: `colcon build --symlink-install`

### 4-Terminal Setup

**Terminal 1 - ArduCopter SITL at Kerala Location:**

```bash
cd ~/Documents/ROSArkairo
./start_sitl_kerala.sh
```

✅ Wait for: `APM: EKF2 IMU0 is using GPS` and `Ready to FLY`

_This starts SITL at your home location (10.047833°N, 76.330278°E) so the drone is already near mission waypoints._

**🔧 In the MAVProxy console (SITL window), disable horizon check for indoor SITL testing:**

```
param set AHRS_EKF_TYPE 3
param set ARMING_CHECK 0
```

This disables arming checks for SITL. **DO NOT use on real hardware!**

---

**Terminal 2 - MAVROS:**

```bash
cd ~/Documents/ROSArkairo/drone1_ws
source install/setup.zsh  # or setup.bash
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://:14550@127.0.0.1:14555 \
  -p gcs_url:=udp://@127.0.0.1:14550
```

✅ Wait for: `[INFO] [mavros.sys]: FCU: ArduCopter V4.X.X`

⚠️ **Keep this running - don't press Ctrl+C!**

---

**Terminal 3 - KML Lane Planner:**

```bash
cd ~/Documents/ROSArkairo/drone1_ws
source install/setup.zsh  # or setup.bash
ros2 run kml_lane_planner kml_lane_planner_node --ros-args \
  -p missions_folder:=/home/YOUR_USERNAME/Documents/ROSArkairo/drone1_ws/missions \
  -p kml_filename:=SOE.kml \
  -p lane_spacing_m:=10.0 \
  -p altitude_m:=50.0
```

✅ Wait for:

- `[INFO] Published mission X with X lane segments`
- `[INFO] Waypoints logged to: SOE_waypoints_XXXXXX.waypoints`

---

**Terminal 4 - Navigation Node:**

```bash
cd ~/Documents/ROSArkairo/drone1_ws
source install/setup.zsh  # or setup.bash
ros2 run drone1_navigation drone1_navigation_node --ros-args \
  --params-file src/drone1_navigation/config/navigation_params.yaml
```

**Expected Sequence:**

```
[INFO] FSM: INIT → WAIT_FOR_FCU
[INFO] ★ Mission received: XX waypoints from X lanes
[INFO] FSM: WAIT_FOR_FCU → SET_GUIDED
[INFO] FSM: SET_GUIDED → ARM
[INFO] FSM: ARM → TAKEOFF
[INFO] Takeoff complete at 50.0m relative
[INFO] ★ Starting navigation to XX waypoints
[INFO] FSM: TAKEOFF → NAVIGATE
[INFO] Flying to WP 1/XX: Distance: XX.Xm
[INFO] ★ DETECTION ENABLED - Starting survey ★
[INFO] Publishing setpoint to WP1: lat=XX.XXXXXX, lon=XX.XXXXXX
...
[INFO] ★ ALL WAYPOINTS COMPLETE ★
[INFO] FSM: NAVIGATE → RTL
```

### Monitoring

**Check topics (Terminal 5 - Optional):**

```bash
cd ~/Documents/ROSArkairo/drone1_ws
source install/setup.zsh
ros2 topic list | grep -E "mavros|mission|detection"
ros2 topic echo /drone1/navigation_status
ros2 topic echo /drone1/detection_enable
```

**View generated waypoints:**

```bash
cat ~/Documents/ROSArkairo/drone1_ws/missions/SOE_waypoints_*.waypoints
```

### Troubleshooting

| Issue                | Solution                                                             |
| -------------------- | -------------------------------------------------------------------- |
| "FCU timeout"        | Ensure Terminal 2 (MAVROS) is running with `FCU: ArduCopter` message |
| "No waypoints"       | Restart Terminal 3 (KML planner) after Terminal 4 is running         |
| "Takeoff timeout"    | Wait 30s after SITL starts before running navigation node            |
| Drone not moving     | Check MAVROS topics exist: `ros2 topic list \| grep mavros`          |
| Mission became stale | Restart all terminals - ensure MAVROS stays running                  |

### Test Mission File

The included `missions/SOE.kml` contains a ~100m×100m test field at coordinates:

- Center: 10.0478°N, 76.3303°E (Kerala, India)
- Altitude: 50m AGL
- Pattern: Lawnmower with 10m lane spacing
- Expected waypoints: 18 (9 lanes × 2 endpoints)

---

## 📜 License

Apache 2.0 - See LICENSE file for details.

---

**Made by Shaan Shoukath**

_Built for precision agriculture with safety-first autonomous drone coordination._
