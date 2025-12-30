# Deployment Guide: Simulation & Real Flight

This guide covers testing the multi-drone agricultural system in SITL simulation and deploying to real hardware.

---

## Table of Contents

1. [Autonomous Operation](#autonomous-operation)
2. [Package Structure](#package-structure)
3. [Prerequisites](#prerequisites)
4. [SITL Simulation Setup](#sitl-simulation-setup)
5. [Mock Testing (No SITL)](#mock-testing-no-sitl)
6. [Real Drone Deployment](#real-drone-deployment)
7. [TF-Luna Object Avoidance](#tf-luna-object-avoidance)
8. [Troubleshooting](#troubleshooting)

---

## Autonomous Operation

### System Behavior

> **Both drones operate fully autonomously once a KML file is detected.**

**Drone-1 (Survey):**

1. Monitors `missions/` folder for new KML files
2. Auto-arms and takes off when KML detected
3. Flies lawnmower survey pattern at **22 feet (6.7m)** altitude
4. Enables disease detection when reaching first waypoint
5. Detects diseases and publishes GPS geotags
6. Auto-RTL to home (GPS fix location) when survey complete

**Drone-2 (Sprayer):**

1. Stays IDLE and disarmed until first geotag received
2. Auto-arms and takes off on first geotag
3. Navigates to target, aligns, and sprays
4. Waits 5 seconds for next geotag
5. If new geotag → flies to next target
6. If no geotag after 5s → Auto-RTL

### State Machines

**Drone-1:**

```
IDLE → ARMING → TAKING_OFF → NAVIGATING → MISSION_COMPLETE → RETURNING_HOME → LANDED
```

**Drone-2:**

```
IDLE → ARMING → TAKING_OFF → NAVIGATING → DETECTING → CENTERING → SPRAYING → WAITING_FOR_NEXT → [NAVIGATING or RTL]
```

---

## Package Structure

### Overview

Each workspace contains three types of packages:

| Package Type  | Purpose                            | Example            |
| ------------- | ---------------------------------- | ------------------ |
| `*_msgs`      | Custom ROS2 message definitions    | `drone1_msgs`      |
| `*_bringup`   | Launch files and parameter configs | `drone1_bringup`   |
| Node packages | Actual ROS2 nodes with logic       | `kml_lane_planner` |

### Message Packages (`*_msgs`)

These define custom message types for inter-node communication:

| Package       | Messages                                      | Description                |
| ------------- | --------------------------------------------- | -------------------------- |
| `drone1_msgs` | `Waypoint`, `LaneSegment`, `LaneSegmentArray` | Survey waypoints and lanes |
| `drone2_msgs` | `MissionState`, `SprayerStatus`               | Sprayer mission tracking   |
| `gcs_msgs`    | `MissionCommand`, `TargetStatus`              | GCS coordination messages  |

### Bringup Packages (`*_bringup`)

These contain launch files that start all nodes for each system:

**`drone1_bringup` → `drone1_survey.launch.py`** launches:

- `kml_lane_planner_node` - KML parsing and waypoint generation
- `drone1_navigation_node` - Survey waypoint following
- `image_capture_node` - Camera feed capture
- `detection_and_geotag_node` - Disease detection with GPS
- `telem_tx_node` - Direct telemetry TX to Drone-2

**`drone2_bringup` → `drone2_sprayer.launch.py`** launches:

- `telem_rx_node` - Receives geotags from Drone-1 telemetry
- `gcs_to_d2_downlink_node` - Target validation and dispatch
- `drone2_navigation_node` - Target navigation
- `local_detection_node` - Local disease confirmation
- `centering_controller_node` - PID visual servoing
- `sprayer_control_node` - Spray actuation
- `mission_manager_node` - State machine and reporting

---

## Prerequisites

### Software Requirements

```bash
# ROS2 (Humble/Iron/Jazzy)
sudo apt install ros-${ROS_DISTRO}-desktop

# MAVROS
sudo apt install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras
sudo /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

# Computer Vision
sudo apt install ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-vision-msgs
sudo apt install ros-${ROS_DISTRO}-geographic-msgs

# Python dependencies
pip3 install scipy geopy opencv-python numpy

# Image tools (for testing)
sudo apt install ros-${ROS_DISTRO}-image-tools ros-${ROS_DISTRO}-rqt-image-view

# Vision messages
sudo apt install ros-${ROS_DISTRO}-vision-msgs
```

### Hardware Requirements (Real Flight)

| Component          | Drone-1 (Survey) | Drone-2 (Sprayer) |
| ------------------ | ---------------- | ----------------- |
| Flight Controller  | Cube Orange+     | Cube Orange+      |
| Companion Computer | Raspberry Pi 5   | Raspberry Pi 5    |
| GPS                | Here3/Neo M10    | Here3/Neo M10     |
| Camera             | Pi Camera 3 Wide | Pi Camera 3 Wide  |
| LiDAR              | TF-Luna (front)  | TF-Luna (front)   |
| Spray System       | -                | PWM Pump + Tank   |

> **Note**: TF-Luna connects directly to Cube Orange+ for object avoidance. ROS does NOT touch this sensor.

---

## SITL Simulation Setup

### Step 1: Install ArduPilot SITL

```bash
# Clone ArduPilot
cd ~
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# Install dependencies
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

# Build SITL
./waf configure --board sitl
./waf copter

# Add to PATH
echo 'export PATH=$PATH:~/ardupilot/Tools/autotest' >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Build ROS2 Workspaces

```bash
# Build all workspaces (message packages are built automatically as dependencies)
cd ~/Documents/ROSArkairo/drone1_ws && colcon build --symlink-install
source install/setup.zsh

cd ~/Documents/ROSArkairo/drone2_ws && colcon build --symlink-install
source install/setup.zsh

# Optional: GCS for monitoring only (not required for operation)
cd ~/Documents/ROSArkairo/gcs_ws && colcon build --symlink-install
source install/setup.zsh
```

> **Note**: GCS is optional - drones communicate directly via telemetry.

### Step 3: Launch SITL + MAVROS

**Terminal 1 - ArduPilot SITL:**

```bash
cd ~/ardupilot/ArduCopter

# Default location (KSFO - San Francisco)
sim_vehicle.py -v ArduCopter --console --map -L KSFO

# OR Custom GPS location (recommended for testing):
# Format: -l LAT,LON,ALT,HEADING
sim_vehicle.py -v ArduCopter --console --map -l 10.0478,76.3303,0,0
```

Wait for "Ready to FLY" message.

**Terminal 2 - MAVROS:**

```bash
source /opt/ros/${ROS_DISTRO}/setup.zsh
ros2 launch mavros apm.launch.py fcu_url:=udp://:14550@127.0.0.1:14555
```

Verify MAVROS is connected:

```bash
ros2 topic echo /mavros/state  # Should show connected: true
```

### Step 4: Launch Drone-1 (Survey)

**Terminal 3:**

```bash
cd ~/Documents/ROSArkairo/drone1_ws
source install/setup.zsh
ros2 launch drone1_bringup drone1_survey.launch.py
```

This launches 5 nodes: KML Planner, Navigation, Image Capture, Detection, and GCS Uplink.

> **Tip**: Use `use_sim:=true` for simulation mode:
> `ros2 launch drone1_bringup drone1_survey.launch.py use_sim:=true`

### Step 5: Create Test Mission

```bash
# Create a test KML polygon in the missions folder
cat > ~/Documents/ROSArkairo/drone1_ws/missions/test_field.kml << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <Placemark>
      <name>Test Field</name>
      <Polygon>
        <outerBoundaryIs>
          <LinearRing>
            <coordinates>
              76.3298,10.0473,0
              76.3308,10.0473,0
              76.3308,10.0483,0
              76.3298,10.0483,0
              76.3298,10.0473,0
            </coordinates>
          </LinearRing>
        </outerBoundaryIs>
      </Polygon>
    </Placemark>
  </Document>
</kml>
EOF
```

The lane planner will automatically detect and process the KML file.

### Step 6: Arm and Start Mission

In the MAVProxy console (Terminal 1):

```
mode GUIDED
arm throttle
takeoff 10
```

Monitor the survey:

```bash
ros2 topic echo /mission/lane_segments
ros2 topic echo /drone1/disease_geotag
```

---

## Multi-Vehicle SITL

For testing both drones simultaneously:

### Launch Two SITL Instances

**Terminal 1 - Drone-1 SITL (Port 5760):**

```bash
sim_vehicle.py -v ArduCopter --console -I 0 --out=udp:127.0.0.1:14550
```

**Terminal 2 - Drone-2 SITL (Port 5770):**

```bash
sim_vehicle.py -v ArduCopter --console -I 1 --out=udp:127.0.0.1:14560
```

### Launch MAVROS for Each

**Terminal 3 - MAVROS for Drone-1:**

```bash
ros2 launch mavros apm.launch.py \
  fcu_url:=udp://:14550@127.0.0.1:14555 \
  namespace:=drone1
```

**Terminal 4 - MAVROS for Drone-2:**

```bash
ros2 launch mavros apm.launch.py \
  fcu_url:=udp://:14560@127.0.0.1:14565 \
  namespace:=drone2
```

### Launch All Systems

```bash
# Terminal 5 - Drone-1 (5 nodes)
cd ~/Documents/ROSArkairo/drone1_ws && source install/setup.zsh
ros2 launch drone1_bringup drone1_survey.launch.py

# Terminal 6 - GCS (2 nodes)
cd ~/Documents/ROSArkairo/gcs_ws && source install/setup.zsh
ros2 launch gcs_bringup gcs.launch.py auto_dispatch:=true

# Terminal 7 - Drone-2 (6 nodes)
cd ~/Documents/ROSArkairo/drone2_ws && source install/setup.zsh
ros2 launch drone2_bringup drone2_sprayer.launch.py
```

**Total: 13 nodes across 3 systems**

---

## Mock Testing (No SITL)

For testing node logic without ArduPilot:

### 1. Fake MAVROS Topics

Create a mock launch file or use these publishers:

```bash
# Terminal 1: GPS
ros2 topic pub -r 10 /mavros/global_position/global sensor_msgs/NavSatFix \
  "{header: {frame_id: 'map'}, latitude: 12.9716, longitude: 77.5946, altitude: 100.0}"

# Terminal 2: IMU
ros2 topic pub -r 50 /mavros/imu/data sensor_msgs/Imu \
  "{orientation: {x: 0, y: 0, z: 0, w: 1}}"

# Terminal 3: Pose (altitude)
ros2 topic pub -r 10 /mavros/local_position/pose geometry_msgs/PoseStamped \
  "{pose: {position: {x: 0, y: 0, z: 10.0}}}"

# Terminal 4: State
ros2 topic pub -r 1 /mavros/state mavros_msgs/State \
  "{connected: true, armed: true, guided: true, mode: 'GUIDED'}"
```

### 2. Fake Camera Feed

```bash
# From video file
ros2 run image_tools cam2image --ros-args \
  -p filename:=/path/to/aerial_video.mp4 \
  -r /image:=/camera/image_raw

# Or from webcam
ros2 run image_tools cam2image --ros-args \
  -p device_id:=0 \
  -r /image:=/camera/image_raw
```

### 3. Test Detection Node

```bash
# Launch detection
ros2 run detection_and_geotag detection_and_geotag_node

# View debug output
ros2 run rqt_image_view rqt_image_view /drone1/detection_debug

# Monitor geotags
ros2 topic echo /drone1/disease_geotag
```

---

## Real Drone Deployment

### Hardware Setup

#### Raspberry Pi 5 Configuration

```bash
# Install ROS2 on Pi
# (Follow ROS2 installation for Ubuntu 24.04 ARM64)

# Clone and build workspaces
git clone https://github.com/yourusername/ROSArkairo.git
cd ROSArkairo/drone1_ws && colcon build

# Setup USB camera
sudo usermod -a -G video $USER
```

#### MAVROS Connection to Cube Orange+

Edit `/etc/mavros.yaml`:

```yaml
# For serial connection (TELEM2)
fcu_url: /dev/ttyAMA0:921600

# Or for USB connection
fcu_url: /dev/ttyACM0:115200

# GCS passthrough (optional)
gcs_url: udp://@192.168.1.100:14550
```

### Flight Checklist

#### Pre-Flight

- [ ] **Hardware**

  - [ ] Props removed for ground tests
  - [ ] Battery charged (>80%)
  - [ ] GPS lock acquired (>8 satellites)
  - [ ] Camera focused and secured
  - [ ] Spray tank filled (Drone-2)

- [ ] **Software**

  - [ ] ROS2 nodes launching without errors
  - [ ] MAVROS connected (`ros2 topic echo /mavros/state`)
  - [ ] GPS publishing (`ros2 topic echo /mavros/global_position/global`)
  - [ ] Camera streaming (`ros2 topic hz /camera/image_raw`)

- [ ] **Parameters**
  - [ ] Home position configured
  - [ ] Geofence boundaries set
  - [ ] Altitude limits verified
  - [ ] KML mission loaded

#### Launch Sequence

**1. Connect to Pi via SSH:**

```bash
ssh pi@drone1.local
```

**2. Start MAVROS:**

```bash
ros2 launch mavros apm.launch.py fcu_url:=/dev/ttyAMA0:921600
```

**3. Verify connection:**

```bash
ros2 topic echo /mavros/state
# Should show: connected: true
```

**4. Start mission nodes:**

```bash
ros2 launch drone1_bringup drone1_survey.launch.py
```

**5. Arm via MAVROS (or RC):**

```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 10}"
```

**6. Load mission:**

```bash
cp field_boundary.kml ~/ROSArkairo/drone1_ws/missions/
# Lane planner auto-detects and publishes waypoints
```

### Safety Configuration

#### ArduPilot Parameters (via Mission Planner)

```
# Geofence
FENCE_ENABLE = 1
FENCE_TYPE = 7          # Altitude + Circle + Polygon
FENCE_ALT_MAX = 50      # Maximum altitude (m)
FENCE_RADIUS = 200      # Maximum radius from home (m)
FENCE_ACTION = 1        # RTL on breach

# Failsafes
FS_THR_ENABLE = 1       # Throttle failsafe
FS_THR_VALUE = 975      # PWM threshold
FS_GCS_ENABLE = 1       # GCS failsafe
FS_BATT_ENABLE = 2      # Battery failsafe (RTL)
FS_BATT_VOLTAGE = 10.5  # Low voltage threshold

# RTL
RTL_ALT = 15            # RTL altitude (m)
RTL_SPEED = 5           # RTL speed (m/s)
```

#### ROS2 Node Parameters

```yaml
# detection_params.yaml
min_detection_area: 500 # Increase for fewer false positives
gps_dedup_distance_m: 5.0 # Wider dedup for real flight

# navigation_params.yaml
arrival_radius_m: 3.0 # Larger radius for GPS error
navigation_timeout_sec: 60 # Shorter timeout
```

---

## Monitoring & Debugging

### Real-Time Visualization

```bash
# RViz2 for waypoints
rviz2

# rqt for plots
ros2 run rqt_plot rqt_plot /mavros/local_position/pose/pose/position/z

# Image view
ros2 run rqt_image_view rqt_image_view
```

### Logging

```bash
# Record all topics
ros2 bag record -a -o flight_log_$(date +%Y%m%d_%H%M%S)

# Record specific topics
ros2 bag record \
  /mavros/global_position/global \
  /drone1/disease_geotag \
  /camera/image_raw \
  -o detection_log
```

### Common Issues

| Issue                  | Cause                 | Solution                        |
| ---------------------- | --------------------- | ------------------------------- |
| MAVROS not connecting  | Wrong port/baud       | Check `fcu_url` parameter       |
| No GPS                 | GPS not locked        | Wait for 3D fix (>8 sats)       |
| Detection not working  | Camera not publishing | Check `/camera/image_raw` topic |
| Waypoints not followed | Not in GUIDED mode    | Set mode to GUIDED              |
| Spray not triggering   | Safety check failed   | Verify armed + GUIDED           |

---

## TF-Luna Object Avoidance

> **TF-Luna is handled entirely by ArduPilot on Cube Orange+ - ROS2 does NOT control it.**

### ArduPilot Configuration (Mission Planner)

```
# Serial Port (assumes SERIAL4/TELEM4)
SERIAL4_PROTOCOL = 11           # Lidar
SERIAL4_BAUD = 115              # 115200 baud

# Proximity Sensor
PRX_TYPE = 8                    # TFMini/TF-Luna
PRX1_ORIENT = 0                 # Forward facing (0°)

# Object Avoidance Behavior
OA_TYPE = 1                     # 1=Stop, 2=BendyRuler path planning
OA_DB_OUTPUT = 3                # Send to fence + object database
AVOID_ENABLE = 7                # All avoidance features
AVOID_MARGIN = 2.0              # 2 meter safety buffer
```

### Behavior During Flight

- **Survey (Drone-1)**: ArduPilot will stop/avoid if obstacle detected during lawnmower pattern
- **Spray Mission (Drone-2)**: ArduPilot will stop before reaching target if obstacle in path

### Verify in Mission Planner

1. Connect to flight controller
2. Go to **Config/Tuning → Full Parameter List**
3. Search for `PRX` parameters
4. Verify `PRX_TYPE = 8` and `PRX1_ORIENT = 0`
5. Check **Flight Data → Status** tab for `sonarrange` value

## Quick Reference

### Node Summary by System

| System    | Nodes  | Launch Command                                        |
| --------- | ------ | ----------------------------------------------------- |
| Drone-1   | 5      | `ros2 launch drone1_bringup drone1_survey.launch.py`  |
| GCS       | 2      | `ros2 launch gcs_bringup gcs.launch.py`               |
| Drone-2   | 6      | `ros2 launch drone2_bringup drone2_sprayer.launch.py` |
| **Total** | **13** |                                                       |

### Essential Commands

```bash
# Check node status
ros2 node list

# Monitor topic
ros2 topic echo /drone1/disease_geotag

# Check message rate
ros2 topic hz /camera/image_raw

# Call service
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# List parameters
ros2 param list /detection_and_geotag_node

# View custom message definition
ros2 interface show drone1_msgs/msg/LaneSegmentArray
```

### Emergency Procedures

```bash
# Force RTL
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'RTL'}"

# Force Land
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'LAND'}"

# Disarm (emergency)
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```

---

_Always test extensively in simulation before real flight. Safety first!_
