# Navigation Node Update - Lane-Based Survey with Detection Control

## ✅ Changes Implemented

### 1. **Lane-Based Navigation**

Changed from simple waypoint navigation to full survey lane support:

**Old Approach:**

- Single waypoint via `/mission/waypoint` (PointStamped, local frame)
- Manual waypoint commands only

**New Approach:**

- Survey mission via `/mission/lane_segments` (LaneSegmentArray)
- Automatic lane-by-lane navigation
- Follows start → end for each lane
- Automatic progression through all lanes

### 2. **Global Position Setpoints**

Changed from local frame to global GPS coordinates:

**Old:** `/mavros/setpoint_position/local` (PoseStamped, ENU coordinates)
**New:** `/mavros/setpoint_position/global` (GlobalPositionTarget, GPS lat/lon/alt)

**Why this change?**

- Survey missions use GPS coordinates from KML files
- More intuitive for outdoor operations
- Better for long-distance navigation
- No coordinate frame transformation needed

### 3. **Detection Node Control**

Added automatic detection enable/disable based on mission progress:

**New Topic:** `/drone1/detection_enable` (std_msgs/Bool)

**Behavior:**

- ✅ **DISABLED** during takeoff and transition to first waypoint
- ✅ **ENABLED** when reaching first lane waypoint
- ✅ **STAYS ENABLED** throughout all lane navigation
- ✅ **DISABLED** when mission complete

**Why?**

- Prevents false detections during takeoff/landing
- Saves computation when not surveying
- Ensures detection only runs during actual survey lanes

### 4. **Lane Progress Monitoring**

Added detailed mission progress tracking:

**New Topic:** `/drone1/next_lane_target` (sensor_msgs/NavSatFix)
Publishes current target waypoint for monitoring

**Console Logging:**

```
★ Mission received: 12 lanes, 1523.4m total
Lane 1/12: Start reached → Navigating to End
Lane 1/12: End reached → Transitioning to next lane
★ DETECTION ENABLED - Starting survey scan ★
Lane 2/12: Start reached → Navigating to End
...
★ DETECTION DISABLED - All lanes complete ★
★ SURVEY MISSION COMPLETE ★
```

---

## 📋 Updated Topic Interface

### Subscribers

| Topic                            | Type                           | Purpose                      |
| -------------------------------- | ------------------------------ | ---------------------------- |
| `/mavros/state`                  | `mavros_msgs/State`            | FCU connection, mode, armed  |
| `/mavros/local_position/pose`    | `geometry_msgs/PoseStamped`    | Local position (for takeoff) |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix`        | Current GPS position         |
| `/mission/lane_segments`         | `drone1_msgs/LaneSegmentArray` | Survey mission lanes         |

### Publishers

| Topic                              | Type                               | Purpose                            |
| ---------------------------------- | ---------------------------------- | ---------------------------------- |
| `/mavros/setpoint_position/global` | `mavros_msgs/GlobalPositionTarget` | Continuous GPS setpoints (10Hz)    |
| `/drone1/next_lane_target`         | `sensor_msgs/NavSatFix`            | Current target waypoint            |
| `/drone1/navigation_status`        | `std_msgs/String`                  | FSM state                          |
| `/drone1/detection_enable`         | `std_msgs/Bool`                    | **NEW** - Enable/disable detection |

---

## 🎯 Mission Flow

```
1. Node starts → INIT → WAIT_FOR_FCU

2. Receive /mission/lane_segments
   ├─ Load lanes: [Lane 1, Lane 2, ..., Lane N]
   └─ Set current_lane_idx = 0, at_lane_start = True

3. FCU connected → SET_GUIDED → ARM → TAKEOFF
   ├─ Detection: DISABLED (not surveying yet)
   └─ Climb to takeoff_altitude_m

4. NAVIGATE → Lane 1 Start
   ├─ Fly to Lane 1 start waypoint
   └─ When reached:
       ├─ ★ DETECTION ENABLED (first waypoint reached)
       └─ Set target to Lane 1 end

5. NAVIGATE → Lane 1 End
   ├─ Fly along lane to end waypoint
   └─ When reached:
       └─ Move to Lane 2

6. Repeat for all lanes...
   ├─ Lane 2: Start → End
   ├─ Lane 3: Start → End
   └─ ...

7. All lanes complete
   ├─ ★ DETECTION DISABLED (mission done)
   ├─ Log: "SURVEY MISSION COMPLETE"
   └─ RTL (Return to Launch)
```

---

## 🔧 Integration with Detection Node

The detection node should subscribe to `/drone1/detection_enable`:

```python
class DetectionNode(Node):
    def __init__(self):
        self.detection_active = False

        self.enable_sub = self.create_subscription(
            Bool,
            '/drone1/detection_enable',
            self.enable_callback,
            10
        )

    def enable_callback(self, msg: Bool):
        if msg.data and not self.detection_active:
            self.get_logger().info('Detection ENABLED')
            self.detection_active = True
            # Start processing images
        elif not msg.data and self.detection_active:
            self.get_logger().info('Detection DISABLED')
            self.detection_active = False
            # Stop processing images

    def image_callback(self, msg):
        if not self.detection_active:
            return  # Skip processing

        # Process image for weed detection
        ...
```

---

## 📊 Example Mission

**Input:** KML file with 4 survey lanes

```yaml
lane_segments:
  - lane_id: 1
    start_waypoint: { lat: 37.7749, lon: -122.4194, alt: 50.0 }
    end_waypoint: { lat: 37.7751, lon: -122.4194, alt: 50.0 }
  - lane_id: 2
    start_waypoint: { lat: 37.7751, lon: -122.4196, alt: 50.0 }
    end_waypoint: { lat: 37.7749, lon: -122.4196, alt: 50.0 }
  - lane_id: 3
    start_waypoint: { lat: 37.7749, lon: -122.4198, alt: 50.0 }
    end_waypoint: { lat: 37.7751, lon: -122.4198, alt: 50.0 }
  - lane_id: 4
    start_waypoint: { lat: 37.7751, lon: -122.4200, alt: 50.0 }
    end_waypoint: { lat: 37.7749, lon: -122.4200, alt: 50.0 }
```

**Execution:**

```
[Takeoff to 50m]
  detection_enable: False

→ Lane 1 Start (37.7749, -122.4194)
  ★ detection_enable: True (first waypoint!)

→ Lane 1 End (37.7751, -122.4194)

→ Lane 2 Start (37.7751, -122.4196)

→ Lane 2 End (37.7749, -122.4196)

→ Lane 3 Start (37.7749, -122.4198)

→ Lane 3 End (37.7751, -122.4198)

→ Lane 4 Start (37.7751, -122.4200)

→ Lane 4 End (37.7749, -122.4200)
  ★ detection_enable: False (mission complete)

[RTL and Land]
```

---

## 🧪 Testing the Detection Control

### Monitor Detection Enable Status

```bash
# Watch detection enable state
ros2 topic echo /drone1/detection_enable

# Expected output:
data: false  # During takeoff
data: false  # Flying to first waypoint
data: true   # ★ Reached first waypoint!
data: true   # Throughout survey
data: false  # ★ Mission complete
```

### Monitor Lane Progress

```bash
# Watch current target
ros2 topic echo /drone1/next_lane_target

# Expected output shows progression:
latitude: 37.7749
longitude: -122.4194
altitude: 50.0
---
latitude: 37.7751
longitude: -122.4194
altitude: 50.0
---
...
```

### Send Test Mission

```bash
# Publish a simple 2-lane test mission
ros2 topic pub /mission/lane_segments drone1_msgs/LaneSegmentArray \
'{
  header: {frame_id: "map"},
  mission_id: "test_mission",
  lanes: [
    {
      lane_id: 1,
      start_waypoint: {latitude: 37.7749, longitude: -122.4194, altitude: 50.0},
      end_waypoint: {latitude: 37.7751, longitude: -122.4194, altitude: 50.0}
    },
    {
      lane_id: 2,
      start_waypoint: {latitude: 37.7751, longitude: -122.4196, altitude: 50.0},
      end_waypoint: {latitude: 37.7749, longitude: -122.4196, altitude: 50.0}
    }
  ],
  total_lanes: 2,
  total_distance_m: 400.0
}'
```

---

## 🚫 Why No LiDAR Subscriber?

**Answer:** Obstacle avoidance is handled by ArduPilot's built-in proximity sensors, not by the companion computer.

### ArduPilot Built-in Avoidance

ArduPilot supports multiple proximity sensors:

- **LiDAR** (TFMini, Benewake, Lightware, etc.)
- **Radar** (Ainstein)
- **Ultrasonic** sensors
- **Realsense** depth cameras

These are connected directly to the Cube Orange+ via I2C, Serial, or CAN bus.

### How It Works

```
LiDAR Sensor (e.g., TFMini)
    ↓ (Serial/I2C)
Cube Orange+ Flight Controller
    ↓
ArduPilot Proximity Library
    ↓
SIMPLE_AVOID or ADSB_AVOID
    ↓
Automatically modifies setpoints to avoid obstacles
```

**The companion computer never sees the raw LiDAR data.**

### Configuration Example

In ArduPilot parameters:

```
PRX_TYPE = 4           # Lightware SF45 rotating lidar
AVOID_ENABLE = 7       # Enable all avoidance
AVOID_MARGIN = 2.0     # 2m margin
AVOID_BEHAVE = 0       # Slide around obstacles
```

ArduPilot will automatically:

- Detect obstacles in the flight path
- Modify navigation commands to avoid them
- Slow down or stop if path blocked
- Resume normal navigation when clear

**Our navigation node doesn't need to know about obstacles** - ArduPilot handles it autonomously at the low level.

---

## ✅ Summary of Changes

| Feature               | Old                               | New                                             |
| --------------------- | --------------------------------- | ----------------------------------------------- |
| **Navigation input**  | Single waypoint                   | Full lane mission                               |
| **Setpoint frame**    | Local (ENU)                       | Global (GPS)                                    |
| **Setpoint topic**    | `/mavros/setpoint_position/local` | `/mavros/setpoint_position/global`              |
| **Detection control** | None                              | `/drone1/detection_enable`                      |
| **Lane tracking**     | None                              | Automatic start→end progression                 |
| **Mission progress**  | None                              | `/drone1/next_lane_target`                      |
| **Avoidance**         | N/A                               | ArduPilot built-in (no LiDAR subscriber needed) |

---

## 🚀 Ready for Integration

The navigation node now:

- ✅ Follows survey lanes automatically
- ✅ Enables detection at the right time
- ✅ Uses global GPS setpoints (matches KML mission format)
- ✅ Publishes lane progress for monitoring
- ✅ Relies on ArduPilot for obstacle avoidance (no LiDAR subscriber needed)

**Build successful** - Ready for testing with lane mission planner!
