# Detection and Geotag Node - Developer Documentation

## Overview

**File**: `drone1_ws/src/detection_and_geotag/detection_and_geotag/detection_and_geotag_node.py`  
**Package**: `detection_and_geotag`  
**Node Name**: `detection_and_geotag_node`  
**Purpose**: Detects yellow disease in plants and computes GPS coordinates of infected areas

## What This Node Does

Computer vision + GPS geotagging pipeline:

1. **Receives camera frames** from image capture node
2. **Detects yellow disease** using HSV color filtering
3. **Validates detections** with green vegetation context
4. **Filters false positives** (sand, soil, lighting artifacts)
5. **Computes GPS coordinates** using ray-casting from drone position
6. **Publishes geotags** to telemetry TX node
7. **Deduplicates detections** to avoid spam

## Core Detection Logic

### HSV Color Space Detection

**Why HSV instead of RGB?**

- **Separates color from brightness**: Yellow hue remains consistent under varying light
- **Simple thresholding**: Single hue range captures all yellows
- **Robust to shadows**: Value channel ignored

```python
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
yellow_mask = cv2.inRange(hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))
```

**Parameter Ranges**:

- **Hue**: 15-40° (yellow range on color wheel)
- **Saturation**: 40-255 (pure color, not faded)
- **Value**: 40-255 (not too dark)

### Multi-Stage Validation Pipeline

**Stage 1: Yellow Detection**

```python
yellow_contours = find_contours(yellow_mask)
```

**Stage 2: Green Context Validation**

```python
for detection in yellow_contours:
    surrounding_region = expand_bbox(detection, 1.5x)
    green_percentage = count_green_pixels(surrounding_region) / total_pixels
    if green_percentage > threshold:
        valid_detections.append(detection)
```

**Reasoning**: Real plant disease occurs on green vegetation, not on bare ground.

**Stage 3: Shape Filtering**

```python
aspect_ratio = width / height
compactness = 4 * pi * area / perimeter²
if 0.3 < aspect_ratio < 3.0 and compactness > 0.4:
    keep_detection()
```

**Purpose**: Eliminates elongated artifacts and irregular noise.

**Stage 4: Size Filtering**

```python
if min_area < contour_area < max_area:
    valid_detection()
```

**Purpose**: Removes tiny noise pixels and unrealistically large regions.

### Severity Classification

```python
if yellow_percentage > 70%:
    severity = "SEVERE"
elif yellow_percentage > 40%:
    severity = "MODERATE"
else:
    severity = "MILD"
```

## GPS Geotagging Algorithm

### Simplified Approach (Current)

**Method**: Use drone's current GPS position when disease is detected

```python
def _pixel_to_gps(self, px, py, frame_w, frame_h):
    # Simply use the drone's current GPS position
    return self.current_gps.latitude, self.current_gps.longitude
```

**Why simplified?**

- Drone-2's visual servoing handles precision centering (~3cm accuracy)
- Drone-1 geotag only needs to be "close enough" (~5m from GPS)
- Removes complexity: no IMU, no camera intrinsics, no ray-casting

### Deduplication System

**Problem**: Drone passes over same disease spot multiple times

**Solution**: Spatial deduplication

```python
for existing in previous_detections:
    if distance(new_detection, existing) < 5.0:  # 5 meters
        reject_duplicate()
```

**Data Structure**:

```python
self.detected_positions = []  # List of (lat, lon, timestamp)
```

## Subscribers

### 1. `/camera/image_raw` (sensor_msgs/Image)

- **Source**: Image Capture Node
- **Rate**: 30 Hz
- **Purpose**: Raw frames for disease detection
- **Processing**: Converts BGR → HSV for color filtering

### 2. `/mavros/global_position/global` (sensor_msgs/NavSatFix)

- **Source**: MAVROS
- **Purpose**: Drone GPS position for geotagging
- **Critical Fields**: latitude, longitude, altitude

### 3. `/mavros/imu/data` (sensor_msgs/Imu)

- **Source**: MAVROS
- **Purpose**: Drone orientation for ray rotation
- **Critical Field**: `orientation` (quaternion)

### 4. `/mavros/local_position/pose` (geometry_msgs/PoseStamped)

- **Source**: MAVROS
- **Purpose**: Altitude above ground for ray-casting
- **Usage**: `pose.position.z`

### 5. `/drone1/detection_enable` (std_msgs/Bool)

- **Source**: Navigation Node
- **Purpose**: Enable/disable detection during appropriate flight phases
- **Logic**: Only process frames when `True`

## Publishers

### 1. `/drone1/disease_geotag` (geographic_msgs/GeoPointStamped)

- **Trigger**: Valid detection after deduplication
- **Purpose**: Sends GPS coordinates to telemetry TX
- **QoS**: RELIABLE (guaranteed delivery to Drone-2)
- **Message Structure**:
  ```python
  GeoPointStamped(
      header=Header(stamp=timestamp, frame_id='map'),
      position=GeoPoint(
          latitude=detected_lat,
          longitude=detected_lon,
          altitude=navigation_altitude
      )
  )
  ```

### 2. `/drone1/detection_debug` (sensor_msgs/Image)

- **Rate**: Same as input (when enabled)
- **Purpose**: Visualization with bounding boxes and severity labels
- **Format**: BGR8 with OpenCV drawings

## Parameters

| Parameter                  | Type  | Default | Description                    |
| -------------------------- | ----- | ------- | ------------------------------ |
| **Camera Intrinsics**      |       |         |                                |
| `camera_fx`                | float | 430.0   | Focal length X (pixels)        |
| `camera_fy`                | float | 430.0   | Focal length Y (pixels)        |
| `camera_cx`                | float | 640.0   | Principal point X              |
| `camera_cy`                | float | 360.0   | Principal point Y              |
| `image_width`              | int   | 1280    | Frame width                    |
| `image_height`             | int   | 720     | Frame height                   |
| **Camera Orientation**     |       |         |                                |
| `camera_roll`              | float | 0.0     | Roll angle (radians)           |
| `camera_pitch`             | float | 1.5708  | Pitch (90° down)               |
| `camera_yaw`               | float | 0.0     | Yaw angle (radians)            |
| **Yellow HSV Range**       |       |         |                                |
| `yellow_h_min`             | int   | 15      | Hue minimum (0-179)            |
| `yellow_h_max`             | int   | 40      | Hue maximum                    |
| `yellow_s_min`             | int   | 40      | Saturation minimum (0-255)     |
| `yellow_s_max`             | int   | 255     | Saturation maximum             |
| `yellow_v_min`             | int   | 40      | Value minimum (0-255)          |
| `yellow_v_max`             | int   | 255     | Value maximum                  |
| **Green HSV Range**        |       |         |                                |
| `green_h_min`              | int   | 35      | Green hue min                  |
| `green_h_max`              | int   | 90      | Green hue max                  |
| `green_s_min`              | int   | 30      | Green saturation min           |
| `green_v_min`              | int   | 30      | Green value min                |
| **Detection Tuning**       |       |         |                                |
| `min_detection_area`       | int   | 500     | Min contour area (pixels²)     |
| `max_detection_area`       | int   | 50000   | Max contour area               |
| `green_context_threshold`  | float | 0.15    | Required green % around yellow |
| `deduplication_distance_m` | float | 5.0     | GPS proximity threshold        |
| `enable_debug_images`      | bool  | True    | Publish visualization          |

## Key Functions

### `process_image()` - Main Pipeline

```python
def process_image(self, img_msg: Image):
    frame = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
    detections = self.detect_yellow_disease(frame)
    for det in detections:
        geotag = self.compute_gps(det)
        if not self.is_duplicate(geotag):
            self.publish_geotag(geotag)
```

### `detect_yellow_disease()` - Computer Vision

```python
def detect_yellow_disease(self, frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    yellow_mask = self.morphological_cleanup(yellow_mask)
    contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return self.validate_detections(contours, frame)
```

**Built-in Functions**:

- `cv2.cvtColor()` - Color space conversion (BGR → HSV)
- `cv2.inRange()` - Binary threshold mask
- `cv2.morphologyEx()` - Noise reduction (opening/closing)
- `cv2.findContours()` - Extracts shape boundaries
- `cv2.contourArea()` - Computes shape area
- `cv2.boundingRect()` - Gets bounding box
- `cv2.rectangle()` - Draws detection boxes
- `cv2.putText()` - Adds labels

### `compute_gps_from_pixel()` - Geotagging

```python
def compute_gps_from_pixel(self, pixel_x, pixel_y):
    ray_camera = self.pixel_to_camera_ray(pixel_x, pixel_y)
    ray_body = self.rotate_camera_to_body(ray_camera)
    ray_world = self.rotate_body_to_world(ray_body)
    ground_point = self.ray_ground_intersection(ray_world)
    lat, lon = self.ecef_to_gps(ground_point)
    return lat, lon
```

### `is_duplicate()` - Spatial Deduplication

```python
def is_duplicate(self, lat, lon) -> bool:
    for prev_lat, prev_lon in self.detected_positions:
        dist = haversine_distance(lat, lon, prev_lat, prev_lon)
        if dist < self.dedup_distance:
            return True
    return False
```

**Built-in Functions**:

- `math.radians()` - Degree to radian
- `math.sin()`, `math.cos()` - Trigonometry
- `math.sqrt()` - Square root
- `math.atan2()` - Arctangent

## Package Dependencies

### ROS2 Packages

- **rclpy**: Node, QoS, Timer
- **sensor_msgs**: Image, Imu, NavSatFix
- **geometry_msgs**: PoseStamped
- **geographic_msgs**: GeoPointStamped, GeoPoint
- **std_msgs**: Header, Bool
- **cv_bridge**: CvBridge (ROS ↔ OpenCV converter)

### Python Libraries

- **cv2** (OpenCV): Computer vision
  - Color space conversions
  - Contour detection
  - Morphological operations
  - Drawing functions
- **numpy** (`np`): Numerical arrays
  - Image data representation
  - Vector/matrix operations
- **scipy**: Scientific computing
  - `Rotation` - Quaternion rotations
- **math**: Mathematical functions
  - Trigonometry: `sin`, `cos`, `atan2`
  - Conversions: `radians`, `degrees`
  - Basic: `sqrt`, `pi`
- **csv**: CSV file writing (detection logging)
- **os**: File system operations
- **datetime**: Timestamp generation
- **typing**: Type hints
- **dataclasses**: Data structures

## HSV Color Space Explained

**What is HSV?**

- **H**ue: Color angle (0-179° in OpenCV, 0-360° normally)
  - Red: 0°, Yellow: 30°, Green: 60°, Blue: 120°
- **S**aturation: Color purity (0-255)
  - 0 = grayscale, 255 = pure color
- **V**alue: Brightness (0-255)
  - 0 = black, 255 = bright

**Why yellow is 15-40°?**

- Pure yellow: ~30° on HSV wheel
- Range accounts for slight variations (lime-yellow to orange-yellow)

**Conversion**:

```python
cv2.COLOR_BGR2HSV  # OpenCV uses BGR by default!
```

## Detection Algorithm Flowchart

```
Input Frame (BGR)
    ↓
Convert to HSV
    ↓
Apply Yellow Mask (inRange)
    ↓
Morphological Cleanup (remove noise)
    ↓
Find Contours
    ↓
For each contour:
    ├─ Size filter (min/max area) ─── REJECT ───┐
    ├─ Shape filter (aspect ratio)  ─── REJECT ───┤
    ├─ Green context check ──────── REJECT ───┤
    └─ VALID ───┐                                │
                ↓                                │
        Compute GPS Geotag                       │
                ↓                                │
        Check Duplicate? ──── YES ─── REJECT ───┤
                ↓ NO                             │
        Publish Geotag                           │
                ↓                                │
        Add to detection history                 │
                                                 ↓
                                            (Discard)
```

## Testing Checklist

- [ ] Detects yellow leaves in test images
- [ ] Rejects sand/soil (brown/tan colors)
- [ ] Validates green vegetation context
- [ ] Computes GPS coordinates (check on map)
- [ ] Deduplicates repeated detections
- [ ] Debug visualization shows bounding boxes
- [ ] Severity classification appears correct
- [ ] Only processes when detection enabled

## Calibration & Tuning

**Yellow not detected?**

- Adjust HSV ranges using debug images
- Lower `yellow_s_min` for faded colors
- Increase `yellow_h_max` for orange tint

**Too many false positives?**

- Increase `green_context_threshold`
- Increase `min_detection_area`
- Tighten HSV ranges

**GPS inaccurate?**

- Check camera intrinsics calibration
- Verify camera pitch = 90° (pointing straight down)
- Confirm IMU orientation correct

## Simulation Mode Testing

### Detection Test Node (`detection_test_node.py`)

A separate test node is available for calibration and testing without full flight stack:

```bash
# Build and source
cd ~/Documents/ROSArkairo/drone1_ws
colcon build --packages-select detection_and_geotag --symlink-install
source install/setup.zsh

# Simulation mode (mock GPS - no MAVROS needed)
ros2 launch detection_and_geotag detection_test.launch.py use_sim:=true

# Real mode (GPS from Orange Cube+ via MAVROS)
ros2 launch detection_and_geotag detection_test.launch.py use_sim:=false

# With live preview window (requires display)
ros2 launch detection_and_geotag detection_test.launch.py use_sim:=true show_window:=true
```

### HSV Calibration (Headless - Ubuntu Server)

```bash
# Adjust yellow detection range via ROS2 params
ros2 param set /detection_test_node yellow_h_min 20
ros2 param set /detection_test_node yellow_h_max 45
ros2 param set /detection_test_node yellow_s_min 100
ros2 param set /detection_test_node yellow_v_min 100

# Monitor detections
ros2 topic echo /drone1/disease_geotag
```

**On shutdown**: Node prints final HSV values for copying to config.

### Test Parameters (config/test_params.yaml)

| Parameter        | Default   | Description                           |
| ---------------- | --------- | ------------------------------------- |
| `use_sim`        | `true`    | Mock GPS (true) or MAVROS GPS (false) |
| `mock_latitude`  | `10.0478` | Base lat for simulation               |
| `mock_longitude` | `76.3303` | Base lon for simulation               |
| `mock_altitude`  | `15.0`    | Flight altitude for simulation        |
| `show_gui`       | `false`   | OpenCV preview (requires display)     |

---

**Last Updated**: January 1, 2026  
**Maintainer**: Shaan Shoukath
