# Telemetry Hardware Testing Guide

## Physical Setup (2 Radio Pairs: A-A and B-B)

### Hardware Configuration

**Pair A (Primary Test)**

- Radio A1 → USB on Computer 1 (Drone-1 side / TX)
- Radio A2 → USB on Computer 2 (Drone-2 side / RX)

**Pair B (Backup/Validation)**

- Radio B1 → USB on Computer 1 (alternative TX)
- Radio B2 → USB on Computer 2 (alternative RX)

---

## Pre-Test: Identify Serial Ports

### On Computer 1 (TX side)

```zsh
# Before plugging radio
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

# Plug in Radio A1, then check again
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
# Note the new port (e.g., /dev/ttyUSB0)
```

### On Computer 2 (RX side)

```zsh
# Same process for Radio A2
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
# Note the port (e.g., /dev/ttyUSB0)
```

### Check Radio Settings (both radios)

```zsh
# Install screen if needed
sudo apt install screen

# Connect to radio serial console (57600 is common for SiK radios)
screen /dev/ttyUSB0 57600

# Press +++ (wait 1s, don't press Enter)
# You should see "OK"

# Check settings:
ATI5          # Show all parameters
ATS1?         # Serial speed
ATS2?         # Air speed
ATS3?         # NetID (MUST match on both radios)
ATS5?         # TX Power

# To exit screen: Ctrl-A then K, then Y
```

**Critical**: Both Radio A1 and A2 must have:

- Same `NetID` (S3 parameter)
- Same `Air Speed` (S2 parameter)
- Compatible frequencies

---

## Test 1: Raw Serial Loopback (No ROS)

### Computer 1 (TX Radio A1 on /dev/ttyUSB0)

```zsh
# Send test data every second
while true; do
  echo "PING $(date +%s)" > /dev/ttyUSB0
  sleep 1
done
```

### Computer 2 (RX Radio A2 on /dev/ttyUSB0)

```zsh
# Read incoming data
cat /dev/ttyUSB0
```

**Expected**: You should see "PING <timestamp>" messages appearing on Computer 2.

**If this fails**:

- Check NetID matches on both radios
- Check TX power (ATS5=20 is common)
- Reduce distance between radios (< 5m for initial test)
- Check antennas are attached

---

## Test 2: MAVROS Serial Bridge Test

### Computer 1 Setup (TX side)

```zsh
cd /home/shaan-shoukath/Documents/ROSArkairo/drone1_ws
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh

# Launch MAVROS with serial connection to Radio A1
ros2 launch mavros apm.launch \
  fcu_url:=serial:///dev/ttyUSB0:57600 \
  namespace:=drone1 \
  tgt_system:=1 \
  tgt_component:=1
```

### Computer 2 Setup (RX side)

```zsh
cd /home/shaan-shoukath/Documents/ROSArkairo/drone2_ws
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh

# Launch MAVROS with serial connection to Radio A2
ros2 launch mavros apm.launch \
  fcu_url:=serial:///dev/ttyUSB0:57600 \
  namespace:=drone2 \
  tgt_system:=2 \
  tgt_component:=1
```

### Send Test MAVLink Message (Computer 1)

```zsh
source /opt/ros/jazzy/setup.zsh

# Send a test named_value_float
ros2 topic pub --once /drone1/mavros/named_value_float/send \
  mavros_msgs/msg/NamedValueFloat \
  "{time_boot_ms: 12345, name: 'test', value: 3.14159}"
```

### Monitor Received Messages (Computer 2)

```zsh
source /opt/ros/jazzy/setup.zsh

# Listen for incoming named_value_float
ros2 topic echo /drone2/mavros/named_value_float
```

**Expected**: You should see the test message appear on Computer 2.

---

## Test 3: Full Telemetry System (TX + RX Nodes)

### Terminal Setup

#### Computer 1 (Drone-1 / TX side) - 2 terminals

**Terminal 1A: MAVROS**

```zsh
cd /home/shaan-shoukath/Documents/ROSArkairo/drone1_ws
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh

ros2 launch mavros apm.launch \
  fcu_url:=serial:///dev/ttyUSB0:57600 \
  namespace:=drone1
```

**Terminal 1B: Telem TX Node**

```zsh
cd /home/shaan-shoukath/Documents/ROSArkairo/drone1_ws
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh

ros2 run telem_tx telem_tx_node
```

#### Computer 2 (Drone-2 / RX side) - 3 terminals

**Terminal 2A: MAVROS**

```zsh
cd /home/shaan-shoukath/Documents/ROSArkairo/drone2_ws
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh

ros2 launch mavros apm.launch \
  fcu_url:=serial:///dev/ttyUSB0:57600 \
  namespace:=drone2
```

**Terminal 2B: Telem RX Node**

```zsh
cd /home/shaan-shoukath/Documents/ROSArkairo/drone2_ws
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh

# Set home position for validation (Kerala coords from your setup)
ros2 run telem_rx telem_rx_node --ros-args \
  -p home_latitude:=10.048300 \
  -p home_longitude:=76.331000 \
  -p validate_coordinates:=true \
  -p max_distance_from_home_m:=5000.0 \
  -p override_altitude:=true \
  -p target_altitude_m:=10.0
```

**Terminal 2C: Monitor Targets**

```zsh
source /opt/ros/jazzy/setup.zsh

# Watch for received targets
ros2 topic echo /drone2/target_position
```

### Send Test Geotag (Computer 1, new terminal)

```zsh
cd /home/shaan-shoukath/Documents/ROSArkairo/drone1_ws
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh

# Send a test disease geotag
ros2 topic pub --once /drone1/disease_geotag \
  geographic_msgs/msg/GeoPointStamped \
  "{header: {stamp: now, frame_id: 'map'},
    position: {latitude: 10.048500, longitude: 76.331200, altitude: 50.0}}"
```

### Expected Results

**Computer 1 (Terminal 1B - TX Node)**:

```
[INFO] [telem_tx_node]: Telemetry TX Node initialized
[INFO] [telem_tx_node]: TX: lat=10.048500 lon=76.331200 alt=50.0 seq=1
```

**Computer 2 (Terminal 2B - RX Node)**:

```
[INFO] [telem_rx_node]: Telemetry RX Node initialized
[INFO] [telem_rx_node]: RX: d_lat=10.048500
[INFO] [telem_rx_node]: RX: d_lon=76.331200
[INFO] [telem_rx_node]: RX: d_alt=50.0
[INFO] [telem_rx_node]: Target validated: distance=25.4m from home
[INFO] [telem_rx_node]: Dispatched target: (10.048500, 76.331200, 10.0m)
```

**Computer 2 (Terminal 2C - Target Monitor)**:

```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: map
latitude: 10.048500
longitude: 76.331200
altitude: 10.0 # Overridden by RX node parameter
```

---

## Test 4: Stress Test (Rapid Messages)

### Computer 1 - Rapid Geotag Sender

```zsh
cd /home/shaan-shoukath/Documents/ROSArkairo/drone1_ws
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh

# Send 10 geotags with slight position changes
for i in {1..10}; do
  lat=$(python3 -c "print(10.048300 + $i * 0.0001)")
  lon=$(python3 -c "print(76.331000 + $i * 0.0001)")

  ros2 topic pub --once /drone1/disease_geotag \
    geographic_msgs/msg/GeoPointStamped \
    "{header: {stamp: now, frame_id: 'map'},
      position: {latitude: $lat, longitude: $lon, altitude: 50.0}}"

  echo "Sent geotag $i: lat=$lat lon=$lon"
  sleep 0.5
done
```

### Computer 2 - Count Received

Watch Terminal 2B and 2C logs. You should see 10 targets received and dispatched.

---

## Test 5: Switch to Pair B (Backup Radios)

Repeat Tests 2-4 but use Radio B1 and B2 (different USB ports):

**Computer 1**: Radio B1 → e.g., `/dev/ttyUSB1`
**Computer 2**: Radio B2 → e.g., `/dev/ttyUSB1`

Update `fcu_url` in MAVROS launch:

```zsh
ros2 launch mavros apm.launch \
  fcu_url:=serial:///dev/ttyUSB1:57600 \
  namespace:=drone1
```

---

## Troubleshooting

### No Messages Received

1. **Check radio link quality**

   ```zsh
   # On either computer, check MAVROS diagnostics
   ros2 topic echo /drone1/mavros/radio_status
   # or
   ros2 topic echo /drone2/mavros/radio_status
   ```

   - `rssi` should be > 100 (local)
   - `remrssi` should be > 50 (remote, adjust distance if lower)

2. **Check MAVLink traffic**

   ```zsh
   # On Computer 2
   ros2 topic hz /drone2/mavros/named_value_float
   ```

   If this shows 0 Hz when you send from Computer 1, the radio link is broken.

3. **Serial port permissions**
   ```zsh
   sudo chmod 666 /dev/ttyUSB0
   # Or add user to dialout group permanently:
   sudo usermod -a -G dialout $USER
   # Then logout and login
   ```

### Geotags Rejected

Check RX node logs for:

- `Target rejected: distance=<X>m exceeds max <Y>m` → adjust `max_distance_from_home_m` parameter
- `Invalid coordinates` → check lat/lon are reasonable values
- `Buffer timeout` → TX node not sending all 3 values (lat, lon, alt)

### High Latency

- Reduce `Air Speed` on both radios (lower = more robust, higher = lower latency)
- Check interference: move away from WiFi routers, USB 3.0 devices
- Reduce distance between radios during testing

---

## Success Criteria

✅ **Test 1**: Raw serial loopback works
✅ **Test 2**: MAVROS sees MAVLink named_value_float on both ends
✅ **Test 3**: Full pipeline: geotag → TX → radio → RX → target_position
✅ **Test 4**: All 10 rapid geotags received and dispatched
✅ **Test 5**: Pair B works identically to Pair A

---

## Production Deployment Notes

1. **Radio Pair Assignment**

   - Pair A: Primary mission (longer range, higher power)
   - Pair B: Backup or secondary drone pair

2. **Mounting**

   - Secure radios away from motors/ESCs (interference)
   - Keep antennas vertical and clear of carbon fiber

3. **Power**

   - Radios typically draw 100-500mA at 5V
   - Can power from flight controller TELEM port or BEC

4. **Final Settings** (via serial console before flight)
   ```
   ATS1=57        # Serial speed 57600
   ATS2=64        # Air speed 64kbps (balance of range/latency)
   ATS3=25        # NetID (unique per pair)
   ATS5=20        # TX Power 20dBm (100mW)
   AT&W           # Write settings to EEPROM
   ATZ            # Reboot radio
   ```

---

## Quick Reference: Commands

```zsh
# Check serial ports
ls /dev/ttyUSB* /dev/ttyACM*

# Test serial at baud 57600
screen /dev/ttyUSB0 57600

# Send test geotag
ros2 topic pub --once /drone1/disease_geotag geographic_msgs/msg/GeoPointStamped \
  "{header: {stamp: now, frame_id: 'map'}, position: {latitude: 10.048500, longitude: 76.331200, altitude: 50.0}}"

# Monitor received targets
ros2 topic echo /drone2/target_position

# Check radio status
ros2 topic echo /drone2/mavros/radio_status

# Monitor MAVLink traffic rate
ros2 topic hz /drone2/mavros/named_value_float
```
