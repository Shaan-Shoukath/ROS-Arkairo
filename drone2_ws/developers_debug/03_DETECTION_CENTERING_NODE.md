# Detection and Centering Node - Developer Documentation

## Overview

**File**: `drone2_ws/src/local_detection_and_centering/local_detection_and_centering/detection_centering_node.py`  
**Package**: `local_detection_and_centering`  
**Node Name**: `detection_centering_node`  
**Purpose**: **MERGED** local disease detection + visual servoing centering

## What This Node Does

Two-phase precision operation:

1. **DETECTION PHASE**: Searches for yellow disease in camera frame
2. **CENTERING PHASE**: Visual servoing to position disease at image center
3. **Publishes spray_ready** when perfectly aligned

## Why Merged?

**Old Architecture**: Separate detection → centering nodes  
**Problem**: Latency, coordination complexity, state synchronization  
**New Solution**: Single node handles both phases seamlessly

**Benefits**:

- Faster response (no topic hops)
- Shared image processing
- Simplified state management
- Direct detection → centering transition

## State Machine

```
IDLE → DETECTING → CENTERING → COMPLETED
  ↑                               ↓
  └──────────────────────────────┘
        (on new arrival)
```

## Core Logic

### State: DETECTING

**Trigger**: `/drone2/arrival_status = True`

**Process**:

```python
1. Capture frame from camera
2. Apply yellow HSV filter
3. Find largest contour
4. Validate size and shape
5. If found → CENTERING
6. If not found and timeout → Report failure
```

**Consecutive Detection**: Requires N consecutive frames with detection to avoid false positives.

### State: CENTERING

**Goal**: Move drone so detection is at image center

**Visual Servoing Algorithm**:

```python
error_x = detection_center_x - image_center_x
error_y = detection_center_y - image_center_y

# PID control
velocity_x = kp*error_x + ki*integral_x + kd*deriv_x
velocity_y = kp*error_y + ki*integral_y + kd*deriv_y

# Publish velocity command
publish_velocity(velocity_x, velocity_y, 0, 0)
```

**Convergence**: When error < threshold for N consecutive frames → COMPLETED

### PID Controller Explained

**P (Proportional)**: `velocity ∝ error`

- Large error → fast movement
- Small error → slow movement

**I (Integral)**: `velocity ∝ ∫error dt`

- Eliminates steady-state offset
- Compensates for wind drift

**D (Derivative)**: `velocity ∝ d(error)/dt`

- Dampens oscillations
- Smooths approach

**Tuning**:

- Increase Kp: Faster response, may oscillate
- Increase Ki: Better accuracy, slower
- Increase Kd: More stable, may sluggish

## Detection Algorithm

### HSV Yellow Detection

**Same as Drone-1 detection**:

```python
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
yellow_mask = cv2.inRange(hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))
contours = cv2.findContours(yellow_mask, ...)
largest = max(contours, key=cv2.contourArea)
```

**Validation**:

- Size: `min_area < contour_area < max_area`
- Shape: Aspect ratio reasonable
- Consecutive: Same detection N frames in row

## Velocity Control

### Coordinate Frame Mapping

**Image Frame**: Origin top-left, X→right, Y→down  
**Drone Body Frame**: X→forward, Y→left, Z→up

**Mapping**:

```python
# Error in image
error_img_x = target_x - center_x  # Positive = target right
error_img_y = target_y - center_y  # Positive = target down

# Velocity in body frame
vel_body_x = -kp * error_img_y  # Forward/back to center Y
vel_body_y = -kp * error_img_x  # Left/right to center X
vel_body_z = 0.0                # Maintain altitude
```

**Why negatives?** Image Y increases downward, but we want to move forward.

### Safety Limits

```python
velocity = clamp(pid_output, -max_vel, +max_vel)
```

**Default max**: 0.5 m/s (safe indoor speed)

## Subscribers

### 1. `/drone2/arrival_status` (std_msgs/Bool)

- **Source**: Drone2 Navigation Node
- **Purpose**: Trigger to start detection
- **Action**: Transition IDLE → DETECTING

### 2. `/camera/image_raw` (sensor_msgs/Image)

- **Source**: Camera node (USB or simulation)
- **Rate**: ~30 Hz
- **Purpose**: Image frames for detection and centering

### 3. `/mavros/local_position/pose` (geometry_msgs/PoseStamped)

- **Source**: MAVROS
- **Purpose**: Current position (not actively used, for logging)

## Publishers

### 1. `/mavros/setpoint_velocity/cmd_vel` (geometry_msgs/TwistStamped)

- **Rate**: 20 Hz (during CENTERING)
- **Purpose**: Velocity commands for visual servoing
- **Frame**: Body frame (FRD or ENU, depends on MAVROS config)

### 2. `/drone2/spray_ready` (std_msgs/Bool)

- **Trigger**: CENTERING complete (centered)
- **Purpose**: Signals sprayer_control to activate
- **Value**: True = ready, False = failed

### 3. `/drone2/detection_status` (std_msgs/Bool)

- **Purpose**: Logging detection success/failure
- **Not critical**: For monitoring only

## Parameters

| Parameter                        | Default | Description                                       |
| -------------------------------- | ------- | ------------------------------------------------- |
| **Detection**                    |         |                                                   |
| `confidence_threshold`           | 0.6     | Min detection confidence (not used in simple HSV) |
| `detection_timeout_sec`          | 30.0    | Max time to find disease                          |
| `max_detection_attempts`         | 5       | Max tries before giving up                        |
| `require_consecutive_detections` | 2       | Frames needed for confirmation                    |
| `min_detection_area_px`          | 500     | Min contour size (pixels²)                        |
| **Centering**                    |         |                                                   |
| `image_width`                    | 1920    | Frame width for center calc                       |
| `image_height`                   | 1080    | Frame height for center calc                      |
| `centered_threshold_pixels`      | 30      | Error tolerance for "centered"                    |
| `max_velocity_mps`               | 0.5     | Max velocity limit (safety)                       |
| `kp`                             | 0.002   | PID proportional gain                             |
| `ki`                             | 0.0001  | PID integral gain                                 |
| `kd`                             | 0.001   | PID derivative gain                               |
| `centering_timeout_sec`          | 30.0    | Max time to center                                |
| `control_rate_hz`                | 20.0    | Velocity command rate                             |

## Key Functions

### `arrival_callback()` - State Trigger

```python
def arrival_callback(self, msg: Bool):
    if msg.data and self.state == State.IDLE:
        self.state = State.DETECTING
        self.state_start_time = now()
```

### `image_callback()` - Main Processing

```python
def image_callback(self, img_msg):
    frame = bridge.imgmsg_to_cv2(img_msg, "bgr8")

    if state == DETECTING:
        bbox = detect_yellow(frame)
        if bbox:
            consecutive_detections += 1
            if consecutive_detections >= threshold:
                self.state = CENTERING

    elif state == CENTERING:
        error_x, error_y = compute_error(bbox)
        velocity = pid_control(error_x, error_y)
        publish_velocity(velocity)

        if error < centered_threshold:
            self.state = COMPLETED
            publish_spray_ready(True)
```

**Built-in Functions**:

- `cv2.cvtColor()` - Color conversion
- `cv2.inRange()` - Threshold mask
- `cv2.findContours()` - Extract shapes
- `cv2.contourArea()` - Shape size
- `cv2.moments()` - Shape centroid
- `cv2.rectangle()` - Draw bounding box

### `pid_control()` - PID Computation

```python
def pid_control(error_x, error_y, dt):
    # Proportional
    p_x = kp * error_x
    p_y = kp * error_y

    # Integral
    integral_x += error_x * dt
    integral_y += error_y * dt
    i_x = ki * integral_x
    i_y = ki * integral_y

    # Derivative
    d_x = kd * (error_x - prev_error_x) / dt
    d_y = kd * (error_y - prev_error_y) / dt

    # Total
    vel_x = clamp(p_x + i_x + d_x, -max_vel, max_vel)
    vel_y = clamp(p_y + i_y + d_y, -max_vel, max_vel)

    return vel_x, vel_y
```

**Built-in**: `max()`, `min()` for clamping

### `detect_yellow()` - Disease Detection

```python
def detect_yellow(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    if area < min_area:
        return None

    M = cv2.moments(largest)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    x, y, w, h = cv2.boundingRect(largest)

    return (cx, cy, x, y, w, h)
```

## Package Dependencies

### ROS2 Packages

- **rclpy**: Node, QoS, Timer
- **sensor_msgs**: Image
- **geometry_msgs**: TwistStamped, PoseStamped
- **std_msgs**: Bool, Header
- **cv_bridge**: CvBridge (OpenCV ↔ ROS)

### Python Libraries

- **cv2** (OpenCV): Computer vision
  - Color space conversion
  - Thresholding
  - Contour detection
  - Drawing functions
- **numpy**: Array operations
- **typing**: Type hints
- **dataclasses**: PIDState structure
- **enum**: State machine (Enum, auto)

## Flowchart

```
Arrival Trigger
      ↓
   DETECTING
      ↓
  [Camera frame]
      ↓
  Yellow HSV filter
      ↓
  Find contours
      ↓
  Largest valid? ─NO→ Retry (timeout check)
      ↓ YES
  Consecutive N frames?
      ↓ YES
   CENTERING
      ↓
  [Camera frame]
      ↓
  Calculate error from center
      ↓
  PID controller
      ↓
  Publish velocity commands
      ↓
  Error < threshold? ─NO→ Continue (timeout check)
      ↓ YES
   COMPLETED
      ↓
  Publish spray_ready = True
      ↓
  Return to IDLE
```

## Testing Checklist

- [ ] Detection phase finds yellow objects
- [ ] Centering moves drone to center detection
- [ ] PID controller doesn't oscillate
- [ ] Velocity limits respected
- [ ] Publishes spray_ready when centered
- [ ] Handles detection failure gracefully
- [ ] Timeouts work correctly

## Tuning Guide

**Detection not working?**

- Check HSV ranges with debug images
- Lower `min_detection_area_px`
- Increase `detection_timeout_sec`

**Centering oscillates?**

- Decrease Kp
- Increase Kd
- Check for delay in image processing

**Too slow to center?**

- Increase Kp
- Increase `max_velocity_mps` (carefully)
- Decrease `centered_threshold_pixels`

**Wind drift issues?**

- Increase Ki to compensate
- Tighten `centered_threshold_pixels`

---

**Last Updated**: December 30, 2025  
**Note**: MERGED node - replaces separate detection + centering nodes
