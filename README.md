# Multi-Drone Agricultural System

**Made by Shaan Shoukath**

A production-grade ROS 2 dual-drone system for **fully autonomous** field surveying and precision spraying with **direct telemetry communication** between drones.

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)
![License](https://img.shields.io/badge/License-Apache%202.0-green)
![Packages](https://img.shields.io/badge/Packages-19-orange)
![Autonomous](https://img.shields.io/badge/Mode-Fully%20Autonomous-brightgreen)

---

## 🏗️ Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    FULLY AUTONOMOUS OPERATION                           │
│                    (Direct Telemetry - GCS-Free)                        │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  Drone-1 (Survey)                          Drone-2 (Sprayer)           │
│  ┌───────────────┐                    ┌─────────────────────┐         │
│  │ KML→Auto-Arm  │                    │ IDLE until geotag  │         │
│  │ Takeoff       │                    │ Auto-Arm on 1st    │         │
│  │ Survey        │═══TELEMETRY═══════►│ Navigate→Spray     │         │
│  │ Detect→Geotag │    (MAVLink)       │ 5s Wait→Next/RTL   │         │
│  │ RTL           │                    │                     │         │
│  └───────────────┘                    └─────────────────────┘         │
│                                                                         │
│  ═══════════════ DIRECT SiK RADIO LINK (No GCS) ═══════════════════   │
│                                                                         │
│  🛡️ TF-Luna Object Avoidance: Handled by ArduPilot (Cube Orange+)      │
└─────────────────────────────────────────────────────────────────────────┘
```

### Direct Telemetry Data Flow

```
Drone-1                                              Drone-2
┌─────────────────────┐                    ┌─────────────────────┐
│ detection_and_      │                    │ telem_rx_node       │
│ geotag_node         │                    │   ↓                 │
│   ↓                 │                    │ gcs_to_d2_downlink  │
│ /drone1/disease_    │    MAVLink         │   ↓                 │
│ geotag              │    NAMED_VALUE_    │ /drone2/target_     │
│   ↓                 │    FLOAT           │ geotag              │
│ telem_tx_node       │═══════════════════►│   ↓                 │
│   ↓                 │    (d_lat,d_lon,   │ Navigation→Spray    │
│ MAVROS              │     d_alt)         │                     │
└─────────────────────┘                    └─────────────────────┘
```

---

## 📁 Workspace Structure

| Workspace   | Packages | Description                                                        |
| ----------- | -------- | ------------------------------------------------------------------ |
| `drone1_ws` | 8        | Survey drone - KML planning, navigation, detection, telemetry TX   |
| `drone2_ws` | 9        | Sprayer drone - telemetry RX, navigation, centering, spray control |
| `gcs_ws`    | 4        | Ground Control Station - _optional monitoring only_                |

> **Note**: GCS is now optional - drones communicate directly via telemetry.

---

## 📖 Node Documentation

### Drone-1 (Survey System)

| Node                     | Description                              | Documentation                                             |
| ------------------------ | ---------------------------------------- | --------------------------------------------------------- |
| **KML Lane Planner**     | Converts KML boundaries to survey paths  | [📄 README](drone1_ws/src/kml_lane_planner/README.md)     |
| **Drone-1 Navigation**   | Executes survey waypoints via MAVROS     | [📄 README](drone1_ws/src/drone1_navigation/README.md)    |
| **Detection & Geotag**   | Disease detection with GPS ray-casting   | [📄 README](drone1_ws/src/detection_and_geotag/README.md) |
| **Telemetry TX** _(NEW)_ | Transmits geotags over MAVLink telemetry | [📄 README](drone1_ws/src/telem_tx/README.md)             |

### Drone-2 (Sprayer System)

| Node                     | Description                          | Documentation                                             |
| ------------------------ | ------------------------------------ | --------------------------------------------------------- |
| **Telemetry RX** _(NEW)_ | Receives geotags from telemetry      | [📄 README](drone2_ws/src/telem_rx/README.md)             |
| **GCS Downlink**         | Validates and dispatches targets     | [📄 README](drone2_ws/src/gcs_to_d2_downlink/README.md)   |
| **Drone-2 Navigation**   | Navigates to target locations        | [📄 README](drone2_ws/src/drone2_navigation/README.md)    |
| **Local Detection**      | Confirms disease presence on arrival | [📄 README](drone2_ws/src/local_detection/README.md)      |
| **Centering Controller** | PID visual servoing for alignment    | [📄 README](drone2_ws/src/centering_controller/README.md) |
| **Sprayer Control**      | PWM actuation with safety checks     | [📄 README](drone2_ws/src/sprayer_control/README.md)      |
| **Mission Manager**      | State machine and status reporting   | [📄 README](drone2_ws/src/mission_manager/README.md)      |

### Ground Control Station _(Optional)_

| Node                | Description                            | Documentation                                         |
| ------------------- | -------------------------------------- | ----------------------------------------------------- |
| **Target Receiver** | Validates and filters incoming targets | [📄 README](gcs_ws/src/gcs_target_receiver/README.md) |
| **Mission Router**  | Dispatches missions (legacy mode)      | [📄 README](gcs_ws/src/gcs_mission_router/README.md)  |

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

#### 🔬 Detection and Geotag Node

Processes camera frames to detect crop disease (yellow spots) and compute GPS coordinates:

- Subscribes to camera images, GPS, IMU, and altitude data
- Performs HSV color thresholding to detect yellow disease patches
- Converts pixel coordinates to GPS using ray-casting (IMU orientation + camera intrinsics)
- De-duplicates detections within configurable radius
- Logs all detections to CSV file

**Key Features**: Configurable HSV thresholds, severity classification, duplicate filtering, debug visualization

---

#### 📡 Telemetry TX Node _(NEW)_

Transmits disease geotags directly to Drone-2 over MAVLink telemetry (SiK radio):

- Subscribes to `/drone1/disease_geotag`
- Encodes GPS coordinates as NAMED_VALUE_FLOAT messages:
  - `d_lat` → latitude
  - `d_lon` → longitude
  - `d_alt` → altitude
- Publishes to `/mavros/named_value_float/send`
- MAVROS forwards to autopilot, which transmits over telemetry

**Key Features**: No GCS dependency, direct drone-to-drone communication, preserves all detection data

---

### Drone-2 (Sprayer System)

#### 📡 Telemetry RX Node _(NEW)_

Receives disease geotags from Drone-1 via MAVLink telemetry:

- Subscribes to `/mavros/named_value_float`
- Buffers incoming `d_lat`, `d_lon`, `d_alt` fields
- Reconstructs complete `GeoPointStamped` when all 3 fields received
- Publishes to `/drone2/target_geotag`

**Key Features**: Message buffering with timeout, automatic buffer clearing, seamless integration with existing nodes

---

#### 📥 GCS to D2 Downlink Node

Validates and dispatches target coordinates for navigation:

- Receives target geotags from telemetry RX node (or GCS in legacy mode)
- Validates coordinates and checks distance from current position
- Converts to local `NavSatFix` for navigation

**Key Features**: Coordinate validation, max distance check, altitude override

---

#### 🛫 Drone-2 Navigation Node

Navigates to target location using global position setpoints:

- Subscribes to target coordinates from downlink
- Publishes `GlobalPositionTarget` via MAVROS
- Monitors distance to target using Haversine calculation
- Publishes arrival status when within configurable radius

**Key Features**: Continuous setpoint publishing, timeout protection, arrival detection

---

#### 📊 Mission Manager Node

**FULLY AUTONOMOUS** - Manages complete mission lifecycle with smart sequencing:

- **Stays IDLE** until first geotag received (never arms without mission)
- **Auto-arms and takes off** on first geotag
- Tracks state: `IDLE → ARMING → TAKING_OFF → NAVIGATING → DETECTING → CENTERING → SPRAYING → WAITING_FOR_NEXT`
- **5-second wait window** after each spray for next geotag
- **Auto-RTL** if no geotag received within wait window

**Key Features**: Configurable `wait_timeout_sec` (default 5s), MAVROS service integration, deterministic behavior

---

## 🚀 Quick Start

> **📖 For complete simulation and real flight instructions, see [DEPLOYMENT.md](DEPLOYMENT.md)**

### Prerequisites

```zsh
# ROS2 dependencies
sudo apt install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras
sudo apt install ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-vision-msgs
sudo apt install ros-${ROS_DISTRO}-geographic-msgs

# Python dependencies
pip3 install scipy geopy
```

### Build All Workspaces

```zsh
# Drone-1
cd ~/Documents/ROSArkairo/drone1_ws
colcon build --symlink-install
source install/setup.zsh

# Drone-2
cd ~/Documents/ROSArkairo/drone2_ws
colcon build --symlink-install
source install/setup.zsh

# GCS (optional - for monitoring only)
cd ~/Documents/ROSArkairo/gcs_ws
colcon build --symlink-install
source install/setup.zsh
```

### Launch Systems (Direct Telemetry Mode)

```zsh
# Terminal 1 - Drone-1 Survey (includes telemetry TX)
ros2 launch drone1_bringup drone1_survey.launch.py

# Terminal 2 - Drone-2 Sprayer (includes telemetry RX)
ros2 launch drone2_bringup drone2_sprayer.launch.py

# NO GCS REQUIRED!
```

### Run a Mission (Fully Autonomous)

```zsh
# Drop KML file to start - everything else is automatic!
cp field_boundary.kml ~/Documents/ROSArkairo/drone1_ws/missions/

# Drone-1 will: Arm → Takeoff (22ft) → Survey → Detect → Transmit geotags → RTL
# Drone-2 will: Wait → Receive geotag → Arm → Fly → Spray → Wait 5s → Next/RTL
```

---

## 📡 Topic Reference

### Drone-1 → Drone-2 (Direct Telemetry)

| Topic                            | Direction | Type            | Description           |
| -------------------------------- | --------- | --------------- | --------------------- |
| `/drone1/disease_geotag`         | D1 Local  | GeoPointStamped | Detected disease      |
| `/mavros/named_value_float/send` | D1→Telem  | NamedValueFloat | MAVLink transport     |
| `/mavros/named_value_float`      | Telem→D2  | NamedValueFloat | Received from Drone-1 |
| `/drone2/target_geotag`          | D2 Local  | GeoPointStamped | Reconstructed target  |

### Drone-2 Internal

| Topic                         | Type      | Description       |
| ----------------------------- | --------- | ----------------- |
| `/drone2/target_position`     | NavSatFix | Navigation target |
| `/drone2/new_target_received` | Bool      | Mission trigger   |
| `/drone2/status`              | String    | Current state     |

---

## 🔧 Key Algorithms

| Algorithm              | Location             | Description                                |
| ---------------------- | -------------------- | ------------------------------------------ |
| **ENU↔GPS Conversion** | Lane Planner         | WGS84 to local metric coordinates          |
| **Ray-Casting GPS**    | Detection Node       | Pixel to GPS using camera intrinsics + IMU |
| **PID Control**        | Centering Controller | Visual servoing for target alignment       |
| **Haversine Distance** | All navigation       | GPS distance calculation                   |
| **MAVLink Encoding**   | Telemetry TX/RX      | GPS to NAMED_VALUE_FLOAT conversion        |

---

## ⚠️ Design Rules

| Rule                                 | Reason                         |
| ------------------------------------ | ------------------------------ |
| ❌ ROS never touches TF-Luna         | Flight controller owns sensors |
| ❌ No UDP/DDS bridges between drones | Prevents network dependency    |
| ✅ MAVROS is the only FC interface   | Standardized MAVLink           |
| ✅ MAVLink for inter-drone comms     | Uses existing telemetry link   |
| ✅ One node = one responsibility     | Clean architecture             |

---

## 📦 Package Summary

| Workspace | Message Pkgs | Node Pkgs | Launch Pkgs |
| --------- | ------------ | --------- | ----------- |
| drone1_ws | 1            | 6         | 1           |
| drone2_ws | 1            | 7         | 1           |
| gcs_ws    | 1            | 2         | 1           |
| **Total** | 3            | 15        | 3           |

---

## 🧪 SITL Testing Guide (Drone-1)

### Prerequisites

- ArduPilot SITL installed (`sim_vehicle.py` available)
- MAVROS installed for ROS 2
- All workspaces built: `colcon build --symlink-install`

### 4-Terminal Setup

**Terminal 1 - ArduCopter SITL at Kerala Location:**

```zsh
cd ~/Documents/ROSArkairo
./start_sitl_kerala.sh
```

✅ Wait for: `APM: EKF2 IMU0 is using GPS` and `Ready to FLY`

**🔧 In the MAVProxy console (SITL window), disable horizon check for indoor SITL testing:**

```
param set AHRS_EKF_TYPE 3
param set ARMING_CHECK 0
```

This disables arming checks for SITL. **DO NOT use on real hardware!**

---

**Terminal 2 - MAVROS:**

```zsh
cd ~/Documents/ROSArkairo/drone1_ws
source install/setup.zsh
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://:14550@127.0.0.1:14555 \
  -p gcs_url:=udp://@127.0.0.1:14550
```

✅ Wait for: `[INFO] [mavros.sys]: FCU: ArduCopter V4.X.X`

---

**Terminal 3 - KML Lane Planner:**

```zsh
cd ~/Documents/ROSArkairo/drone1_ws
source install/setup.zsh
ros2 run kml_lane_planner kml_lane_planner_node --ros-args \
  -p missions_folder:=$HOME/Documents/ROSArkairo/drone1_ws/missions \
  -p kml_filename:=SOE.kml \
  -p lane_spacing_m:=10.0 \
  -p altitude_m:=50.0
```

---

**Terminal 4 - Navigation Node:**

```zsh
cd ~/Documents/ROSArkairo/drone1_ws
source install/setup.zsh
ros2 run drone1_navigation drone1_navigation_node --ros-args \
  --params-file src/drone1_navigation/config/navigation_params.yaml
```

### Monitoring

```zsh
cd ~/Documents/ROSArkairo/drone1_ws
source install/setup.zsh
ros2 topic list | grep -E "mavros|mission|detection"
ros2 topic echo /drone1/navigation_status
ros2 topic echo /drone1/detection_enable
```

### Troubleshooting

| Issue             | Solution                                                             |
| ----------------- | -------------------------------------------------------------------- |
| "FCU timeout"     | Ensure Terminal 2 (MAVROS) is running with `FCU: ArduCopter` message |
| "No waypoints"    | Restart Terminal 3 (KML planner) after Terminal 4 is running         |
| "Takeoff timeout" | Wait 30s after SITL starts before running navigation node            |
| Drone not moving  | Check MAVROS topics exist: `ros2 topic list \| grep mavros`          |

---

## 📜 License

Apache 2.0 - See LICENSE file for details.

---

**Made by Shaan Shoukath**

_Built for precision agriculture with safety-first autonomous drone coordination._
