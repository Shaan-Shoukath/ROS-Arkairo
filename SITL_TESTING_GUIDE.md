# SITL Testing Setup - Updated Drone-2 System

**Test the merged detection_centering node and enhanced mission manager**

---

## Prerequisites

```zsh
# Build the updated system first
cd ~/Documents/ROSArkairo
./build_drone2_updates.sh
source drone2_ws/install/setup.bash
```

---

## 4-Terminal SITL Setup

### 🖥️ Terminal 1: ArduCopter SITL

```zsh
cd ~/Documents/ROSArkairo
./start_sitl_kerala.sh
```

**Wait for**: `APM: EKF2 IMU0 is using GPS` and `Ready to FLY`

**Then in MAVProxy console, disable arming checks for SITL testing**:

```
param set ARMING_CHECK 0
mode guided
```

---

### 🖥️ Terminal 2: MAVROS (Drone-2 Namespace)

```zsh
cd ~/Documents/ROSArkairo/drone2_ws
source install/setup.bash

ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://:14550@127.0.0.1:14555 \
  -p gcs_url:=udp://@127.0.0.1:14550 \
  --remap __ns:=/drone2
```

**Wait for**: `[INFO] [mavros.sys]: FCU: ArduCopter V4.X.X`

---

### 🖥️ Terminal 3: Drone-2 Sprayer System (NEW MERGED NODE)

```zsh
cd ~/Documents/ROSArkairo/drone2_ws
source install/setup.bash

ros2 launch drone2_bringup drone2_sprayer.launch.py
```

**Expected output**:

- `Telemetry RX Node initialized`
- `Drone-2 Navigation Node initialized`
- `Detection & Centering Node initialized (MERGED)` ← NEW!
- `Sprayer Control Node initialized`
- `Drone-2 Mission Manager - FULLY AUTONOMOUS`
- `State: IDLE - Waiting for first geotag`

---

### 🖥️ Terminal 4: Test Commands & Monitoring

```zsh
cd ~/Documents/ROSArkairo/drone2_ws
source install/setup.bash

# Monitor mission state
ros2 topic echo /drone2/status
```

---

## Testing Workflow

### Step 1: Verify System is Running

```zsh
# Check all nodes are alive
ros2 node list | grep -E "telem_rx|navigation|detection_centering|sprayer"

# Expected output:
# /telem_rx_node
# /drone2_navigation_node          ← UNIFIED (includes ARM/TAKEOFF)
# /detection_centering_node        ← MERGED (detection + centering)
# /sprayer_control_node

# Check MAVROS connection
ros2 topic hz /drone2/mavros/state
```

---

### Step 2: Send First Geotag (Triggers Autonomous Sequence)

**In Terminal 4, send first test geotag:**

```zsh
# First geotag - should trigger GUIDED → ARM → TAKEOFF → NAVIGATE
ros2 topic pub /drone2/target_geotag geographic_msgs/msg/GeoPointStamped \
  "{header: {stamp: now, frame_id: 'map'},
    position: {latitude: 10.014, longitude: 76.328, altitude: 10.0}}" \
  --once
```

**Expected state transitions:**

```
IDLE → GUIDED_SWITCH → ARMING → TAKING_OFF → NAVIGATING → DETECTING_CENTERING → SPRAYING → WAITING_FOR_NEXT
```

---

### Step 3: Monitor State Machine

**Open new terminal pane and watch states:**

```zsh
cd ~/Documents/ROSArkairo/drone2_ws
source install/setup.bash

# Watch mission state (should show state transitions)
watch -n 0.5 "ros2 topic echo /drone2/status --once"
```

**Or multiple monitors side-by-side:**

```zsh
# Terminal pane 1: Mission state
ros2 topic echo /drone2/status

# Terminal pane 2: Flight mode
ros2 topic echo /drone2/mavros/state | grep -A2 "mode:"

# Terminal pane 3: Altitude
ros2 topic echo /drone2/mavros/local_position/pose | grep -A3 "position:"

# Terminal pane 4: Detection/Centering status
ros2 topic echo /drone2/spray_ready
```

---

### Step 4: Send Second Geotag (Within 5 Seconds)

**After spray completes, send another geotag within 5 seconds:**

```zsh
# Second geotag - should continue mission
ros2 topic pub /drone2/target_geotag geographic_msgs/msg/GeoPointStamped \
  "{header: {stamp: now, frame_id: 'map'},
    position: {latitude: 10.015, longitude: 76.329, altitude: 10.0}}" \
  --once
```

**Expected**: Goes back to `NAVIGATING` state for second target

---

### Step 5: Test 5-Second Timeout (RTL Behavior)

**After spray completes, DON'T send geotag for 5+ seconds:**

**Expected state transitions:**

```
SPRAYING → WAITING_FOR_NEXT → (5 seconds elapse) → RETURNING_HOME → LANDED → IDLE
```

---

## Detailed Monitoring Commands

### Mission Manager Status

```zsh
# Current state
ros2 topic echo /drone2/status --once

# State history (last 10)
ros2 topic echo /drone2/status | head -n 20
```

### Detection & Centering (MERGED NODE)

```zsh
# Detection status
ros2 topic echo /drone2/detection_status

# Spray ready signal (after centering complete)
ros2 topic echo /drone2/spray_ready

# Velocity commands during centering
ros2 topic echo /drone2/mavros/setpoint_velocity/cmd_vel
```

### Navigation

```zsh
# Current GPS position
ros2 topic echo /drone2/mavros/global_position/global --once

# Target position
ros2 topic echo /drone2/target_position --once

# Arrival status
ros2 topic echo /drone2/arrival_status
```

### Sprayer

```zsh
# Spray done signal
ros2 topic echo /drone2/spray_done

# PWM output
ros2 topic echo /drone2/pwm_spray
```

### Flight Controller

```zsh
# Armed status and mode
ros2 topic echo /drone2/mavros/state --once

# Battery
ros2 topic echo /drone2/mavros/battery

# Altitude
ros2 topic echo /drone2/mavros/altitude
```

---

## Debug Commands

### Check Node Health

```zsh
# List all running nodes
ros2 node list

# Node info for merged detection/centering
ros2 node info /detection_centering_node

# Node info for unified navigation
ros2 node info /drone2_navigation_node
```

### Check Topics

```zsh
# List all drone2 topics
ros2 topic list | grep drone2

# Check topic frequency
ros2 topic hz /drone2/target_geotag
ros2 topic hz /drone2/mavros/state
ros2 topic hz /camera/image_raw  # If using camera
```

### Check Parameters

```zsh
# Detection & centering parameters
ros2 param list /detection_centering_node

# Navigation parameters
ros2 param list /drone2_navigation_node

# Get specific parameter
ros2 param get /detection_centering_node centered_threshold_pixels
ros2 param get /drone2_navigation_node wait_timeout_sec
```

---

## Test Scenarios

### Scenario 1: Single Target Mission

```zsh
# Send one geotag
ros2 topic pub /drone2/target_geotag geographic_msgs/msg/GeoPointStamped \
  "{header: {stamp: now, frame_id: 'map'},
    position: {latitude: 10.014, longitude: 76.328, altitude: 10.0}}" \
  --once

# Wait and observe: IDLE → ... → SPRAYING → WAITING_FOR_NEXT → (5s) → RTL
```

### Scenario 2: Multi-Target Mission

```zsh
# Send first geotag
ros2 topic pub /drone2/target_geotag geographic_msgs/msg/GeoPointStamped \
  "{header: {stamp: now, frame_id: 'map'},
    position: {latitude: 10.014, longitude: 76.328, altitude: 10.0}}" \
  --once

# Wait ~30 seconds for spray to complete, then within 5 seconds send second
ros2 topic pub /drone2/target_geotag geographic_msgs/msg/GeoPointStamped \
  "{header: {stamp: now, frame_id: 'map'},
    position: {latitude: 10.015, longitude: 76.329, altitude: 10.0}}" \
  --once

# Repeat for third target if desired
ros2 topic pub /drone2/target_geotag geographic_msgs/msg/GeoPointStamped \
  "{header: {stamp: now, frame_id: 'map'},
    position: {latitude: 10.016, longitude: 76.330, altitude: 10.0}}" \
  --once
```

### Scenario 3: Test Detection & Centering

```zsh
# Manually trigger detection (requires drone at target)
ros2 topic pub /drone2/arrival_status std_msgs/msg/Bool "data: true" --once

# Watch detection node
ros2 topic echo /drone2/detection_status

# Watch centering
ros2 topic echo /drone2/mavros/setpoint_velocity/cmd_vel

# Watch spray ready
ros2 topic echo /drone2/spray_ready
```

---

## Troubleshooting

### Issue: "No nodes appear"

```zsh
# Rebuild and source
cd ~/Documents/ROSArkairo/drone2_ws
colcon build --symlink-install
source install/setup.bash

# Relaunch
ros2 launch drone2_bringup drone2_sprayer.launch.py
```

### Issue: "Stays in IDLE"

```zsh
# Check if geotag topic exists
ros2 topic list | grep geotag

# Verify topic name is correct
ros2 topic info /drone2/target_geotag

# Try publishing again
ros2 topic pub /drone2/target_geotag geographic_msgs/msg/GeoPointStamped \
  "{header: {stamp: now, frame_id: 'map'},
    position: {latitude: 10.014, longitude: 76.328, altitude: 10.0}}" \
  --once
```

### Issue: "Won't ARM"

```zsh
# Check SITL
# In Terminal 1 (MAVProxy):
param set ARMING_CHECK 0
mode guided

# Check MAVROS connection
ros2 topic echo /drone2/mavros/state --once
```

### Issue: "Detection times out"

```zsh
# Check if camera topic exists (if using real camera)
ros2 topic list | grep camera

# For SITL testing without camera, detection will timeout - this is expected
# The workflow will continue: timeout → skip to next or RTL
```

---

## Success Indicators

✅ **Terminal 1**: SITL shows `Ready to FLY`  
✅ **Terminal 2**: MAVROS shows `FCU: ArduCopter` connected  
✅ **Terminal 3**: All 5 nodes start successfully  
✅ **Terminal 4**: Geotag triggers state change from IDLE  
✅ **Flight mode changes**: RTL → GUIDED (on first geotag)  
✅ **Drone arms**: `armed: true` in `/drone2/mavros/state`  
✅ **Drone takes off**: Altitude increases to ~10m  
✅ **State transitions**: All states reached in sequence  
✅ **5s timeout works**: Goes to RTL if no geotag

---

## Quick Reference - State Machine

```
1. IDLE                  - Waiting for first geotag
2. GUIDED_SWITCH         - Setting GUIDED mode
3. ARMING                - Arming throttle
4. TAKING_OFF            - Climbing to altitude
5. NAVIGATING            - Flying to GPS target
6. DETECTING_CENTERING   - Finding & centering (MERGED!)
7. SPRAYING              - Relay ON
8. WAITING_FOR_NEXT      - 5 second window
9. RETURNING_HOME        - RTL mode
10. LANDED               - On ground
    └─> Back to IDLE
```

---

## Log Files

Logs are stored in:

```
~/Documents/ROSArkairo/drone2_ws/log/
```

View latest build log:

```zsh
cat ~/Documents/ROSArkairo/drone2_ws/log/latest_build/events.log
```

---

## Clean Shutdown

```zsh
# Terminal 3: Ctrl+C (stop ROS nodes)
# Terminal 2: Ctrl+C (stop MAVROS)
# Terminal 1: Ctrl+C (stop SITL)
```

---

**Ready to test! Start with Terminal 1, then 2, then 3, then 4.** 🚁✨
