# Raspberry Pi 5 Complete Setup Guide for Drone-1

This guide walks you through setting up everything on a fresh Raspberry Pi 5 with Ubuntu 24.04 + ROS2 Jazzy + MAVROS already installed.

---

## Table of Contents

1. [System Dependencies](#step-1-system-dependencies)
2. [Python Dependencies](#step-2-python-dependencies)
3. [UART Configuration](#step-3-uart-configuration)
4. [Camera Setup](#step-4-camera-setup-optional)
5. [Clone and Build Workspace](#step-5-clone-and-build-workspace)
6. [GeographicLib Datasets](#step-6-geographiclib-datasets)
7. [ArduPilot Configuration](#step-7-ardupilot-configuration)
8. [Competition Setup](#step-8-competition-setup-start-location)
9. [Systemd Service (Auto-start)](#step-9-systemd-service-auto-start)
10. [Testing](#step-10-testing)

---

## Step 1: System Dependencies

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# ROS2 packages (some may already be installed)
sudo apt install -y \
  ros-jazzy-cv-bridge \
  ros-jazzy-vision-msgs \
  ros-jazzy-geographic-msgs \
  ros-jazzy-image-tools \
  ros-jazzy-rqt-image-view

# Build tools
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  git
```

---

## Step 2: Python Dependencies

```bash
# Core Python libraries
pip3 install --user \
  scipy \
  geopy \
  opencv-python \
  numpy

# For telemetry (if using GCS forwarder)
pip3 install --user pymavlink
```

---

## Step 3: UART Configuration

The Cube Orange+ connects via UART (TELEM2) to the Raspberry Pi GPIO.

### 3.1: Enable UART on Pi 5

```bash
# Edit boot config
sudo nano /boot/firmware/config.txt

# Add these lines at the end:
enable_uart=1
dtoverlay=uart0
```

### 3.2: Disable Serial Console

The default serial console interferes with MAVROS:

```bash
# Stop and disable the console service
sudo systemctl stop serial-getty@ttyAMA0.service
sudo systemctl disable serial-getty@ttyAMA0.service

# Also edit cmdline.txt to remove console
sudo nano /boot/firmware/cmdline.txt
# Remove: console=serial0,115200 (if present)
```

### 3.3: Set Serial Permissions

```bash
# Add your user to dialout group (permanent)
sudo usermod -a -G dialout $USER

# You must logout and login for this to take effect!
# Or temporarily:
sudo chmod 666 /dev/ttyAMA0
```

### 3.4: Reboot

```bash
sudo reboot
```

### 3.5: Verify UART

After reboot:
```bash
# Check the device exists
ls -la /dev/ttyAMA0
# Should show: crw-rw---- 1 root dialout ...

# Test serial permissions
cat /dev/ttyAMA0
# Should hang (waiting for data), not error
# Press Ctrl+C to exit
```

---

## Step 4: Camera Setup (Optional)

If using Pi Camera 3 for detection:

### 4.1: Enable Camera

```bash
# Edit boot config
sudo nano /boot/firmware/config.txt

# Add at the end (for Pi Camera 3):
dtoverlay=imx708

# Or for older Pi Camera v2:
# dtoverlay=imx219
```

### 4.2: Add Camera Permissions

```bash
# Add user to video group
sudo usermod -a -G video $USER
# Logout and login
```

### 4.3: Reboot and Test

```bash
sudo reboot

# After reboot, test camera
libcamera-hello -t 5000
# Should show 5 seconds of preview

# List camera
libcamera-hello --list-cameras
```

---

## Step 5: Clone and Build Workspace

### 5.1: Clone Repository

```bash
# Create workspace directory
mkdir -p ~/Drone
cd ~/Drone

# Clone the repo (replace with your actual repo URL)
git clone https://github.com/Shaan-Shoukath/ROS-Arkairo.git
cd ROS-Arkairo
```

### 5.2: Initialize rosdep

```bash
# Only needed once per system
sudo rosdep init
rosdep update
```

### 5.3: Install ROS Dependencies

```bash
cd ~/Drone/ROS-Arkairo/drone1_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5.4: Build Workspace

```bash
cd ~/Drone/ROS-Arkairo/drone1_ws

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Build
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### 5.5: Add to .bashrc

```bash
# Add these lines to ~/.bashrc for persistence
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/Drone/ROS-Arkairo/drone1_ws/install/setup.bash" >> ~/.bashrc
```

---

## Step 6: GeographicLib Datasets

MAVROS needs geographic datasets for ENU↔GPS conversion:

```bash
# Run the MAVROS installer script
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

# This downloads ~15MB of data to /usr/share/GeographicLib/
# Takes about 1-2 minutes
```

**Verify installation:**
```bash
ls /usr/share/GeographicLib/geoids/
# Should show: egm96-5.pgm or similar
```

---

## Step 7: ArduPilot Configuration

Connect to Cube Orange+ via Mission Planner or QGroundControl and set these parameters:

### 7.1: Serial Port Configuration

```
SERIAL2_PROTOCOL = 2       # MAVLink2 on TELEM2
SERIAL2_BAUD = 921         # 921600 baud (fastest for Pi)
```

### 7.2: System ID

```
SYSID_THISMAV = 1          # This is Drone 1
SYSID_MYGCS = 255          # Default GCS ID
```

### 7.3: MAVLink Forwarding (for telemetry TX)

```
SERIAL1_PROTOCOL = 2       # MAVLink2 on TELEM1 (radio)
SERIAL1_BAUD = 57          # 57600 (typical for SiK radios)
MAV_FORWARD = 1            # CRITICAL: Forward between ports
```

### 7.4: GPS

```
GPS_TYPE = 1               # Auto-detect
GPS_GNSS_MODE = 0          # All constellations
```

### 7.5: Arming (Set for testing, change for flight!)

```
# FOR BENCH TESTING ONLY:
ARMING_CHECK = 0           # Disable all checks

# FOR REAL FLIGHT:
ARMING_CHECK = 1           # Enable all checks
```

### 7.6: Write Parameters

In Mission Planner: **Config → Full Parameter List → Write Params**

---

## Step 8: Competition Setup (Start Location)

Configure the drone's start/home location for your competition field. Edit these YAML files:

### 8.1: Navigation Config

Edit `~/Drone/ROS-Arkairo/drone1_ws/src/drone1_navigation/config/navigation_params.yaml`:

```yaml
# Start Location (Competition Setup)
use_gps_home: true         # true = GPS lock, false = use custom below
start_latitude: 10.0478    # Your field latitude
start_longitude: 76.3303   # Your field longitude
start_altitude_m: 0.0      # Ground altitude (meters)
```

### 8.2: KML Planner Config

Edit `~/Drone/ROS-Arkairo/drone1_ws/src/kml_lane_planner/config/planner_params.yaml`:

```yaml
# Start Location (Competition Setup)
require_gps_home: false    # true = wait for GPS, false = use default
default_home_lat: 10.0478  # Your field latitude
default_home_lon: 76.3303  # Your field longitude
```

### 8.3: Rebuild After Changes

```bash
cd ~/Drone/ROS-Arkairo/drone1_ws
colcon build --symlink-install
```

> [!TIP]
> **For Competition Day:**
> 1. Set `use_gps_home: true` to use live GPS lock
> 2. Or set to `false` and enter exact field coordinates for repeatability

---

## Step 9: Systemd Service (Auto-start)

Create a service to start drone software on boot:

### 8.1: Create Service File

```bash
sudo nano /etc/systemd/system/drone1.service
```

Paste this content:

```ini
[Unit]
Description=Drone-1 ROS2 Survey System
After=network.target

[Service]
Type=simple
User=YOUR_USERNAME
WorkingDirectory=/home/YOUR_USERNAME/Drone/ROS-Arkairo/drone1_ws
Environment="ROS_DOMAIN_ID=0"
ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch drone1_bringup drone1_survey.launch.py'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Replace `YOUR_USERNAME` with your actual username** (e.g., `pi`, `ubuntu`, etc.)

### 8.2: Enable Service

```bash
# Reload systemd
sudo systemctl daemon-reload

# Enable on boot
sudo systemctl enable drone1.service

# Start now (optional)
sudo systemctl start drone1.service

# Check status
sudo systemctl status drone1.service
```

### 8.3: View Logs

```bash
# Follow logs
journalctl -u drone1.service -f

# Last 100 lines
journalctl -u drone1.service -n 100
```

---

## Step 10: Testing

### 10.1: Test MAVROS Connection

```bash
# Terminal 1: Source and launch MAVROS
source ~/Drone/ROS-Arkairo/drone1_ws/install/setup.bash
ros2 launch mavros apm.launch.py fcu_url:=serial:///dev/ttyAMA0:921600

# Wait for:
# [INFO] [mavros.sys]: FCU: ArduCopter V4.x.x
# [INFO] [mavros.sys]: Got HEARTBEAT, connected.
```

### 10.2: Verify GPS

```bash
# Terminal 2:
ros2 topic echo /mavros/global_position/global

# Should see lat/lon/alt updating (take outside for GPS fix)
```

### 10.3: Verify State

```bash
ros2 topic echo /mavros/state

# Should show:
# connected: true
# mode: 'STABILIZE'
```

### 10.4: List All Topics

```bash
ros2 topic list | grep mavros | head -20
```

### 10.5: Test Full System (Props OFF!)

```bash
cd ~/Drone/ROS-Arkairo
./launch_drone1_on_hardware.sh
```

This opens 3 terminals:
1. MAVROS
2. KML Lane Planner
3. Navigation Node

### 10.6: Monitor Navigation

```bash
ros2 topic echo /drone1/navigation_status
```

---

## Quick Reference: Connection Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                  RASPBERRY PI 5 TO CUBE ORANGE+ WIRING                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   Raspberry Pi 5 GPIO                    Cube Orange+ TELEM2                │
│   ┌─────────────────┐                    ┌─────────────────┐                │
│   │ Pin 8 (TX)  ────┼────────────────────┼─► Pin 3 (RX)    │                │
│   │ Pin 10 (RX) ◄───┼────────────────────┼── Pin 2 (TX)    │                │
│   │ Pin 6 (GND) ────┼────────────────────┼── Pin 6 (GND)   │                │
│   │                 │                    │                 │                │
│   │ USB-C Power     │                    │ 5V from Battery │                │
│   └─────────────────┘                    └─────────────────┘                │
│                                                                             │
│   ⚠️ DO NOT CONNECT 5V FROM CUBE TO PI - Power Pi separately!              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| `Permission denied: /dev/ttyAMA0` | Not in dialout group | `sudo usermod -a -G dialout $USER && logout` |
| MAVROS: `FCU timeout` | Wrong baud or port | Check `SERIAL2_BAUD` matches `fcu_url` |
| No GPS data | GPS not connected or no fix | Wait outdoors for fix, check `ros2 topic hz` |
| Build fails on drone1_msgs | Circular dependency | `colcon build --packages-select drone1_msgs` first |
| Camera not found | dt overlay missing | Add `dtoverlay=imx708` to config.txt |

---

## Summary Checklist

- [x] ✅ Ubuntu 24.04 installed
- [x] ✅ ROS2 Jazzy installed  
- [x] ✅ MAVROS installed
- [ ] ⬜ System dependencies (`cv-bridge`, `vision-msgs`, etc.)
- [ ] ⬜ Python dependencies (`scipy`, `geopy`, `opencv-python`)
- [ ] ⬜ UART enabled (`enable_uart=1`)
- [ ] ⬜ Serial console disabled
- [ ] ⬜ User in `dialout` group
- [ ] ⬜ GeographicLib datasets installed
- [ ] ⬜ Workspace cloned and built
- [ ] ⬜ ArduPilot parameters set
- [ ] ⬜ MAVROS connects successfully
- [ ] ⬜ GPS topic publishing

---

**You're ready to fly! 🚁** (After proper safety checks, of course!)
