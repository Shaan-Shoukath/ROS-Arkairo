# Detection and Geotag Node

**Real-time Disease Detection with GPS Ray-Casting**

## What It Does

Processes camera frames during survey flight to detect crop diseases (yellow spots) and converts pixel coordinates to precise GPS geotags using the drone's pose and camera intrinsics.

## Logic Flow

```
Camera Frame Received
        │
        ▼
Convert to HSV Color Space
        │
        ▼
Apply HSV Threshold (detect yellow)
        │
        ▼
Find Contours → Filter by Area
        │
        ▼
For Each Detection:
   ├── Get Bounding Box Center (pixel x, y)
   ├── Ray-Cast to Ground (using altitude + IMU)
   ├── Convert Ground Offset to GPS
   ├── Check Duplicate (within radius)
   └── Publish GeoPointStamped
        │
        ▼
Log to CSV + Optional Debug Visualization
```

## Subscribers

| Topic                            | Type        | Callback                | Description              |
| -------------------------------- | ----------- | ----------------------- | ------------------------ |
| `/camera/image_raw`              | `Image`     | `image_callback()`      | Raw camera frames        |
| `/mavros/global_position/global` | `NavSatFix` | `gps_callback()`        | Current GPS              |
| `/mavros/imu/data`               | `Imu`       | `imu_callback()`        | Orientation              |
| `/mavros/altitude`               | `Altitude`  | `altitude_callback()`   | Height AGL               |
| `/drone1/detection_enable`       | `Bool`      | `detection_enable_cb()` | Enable/disable detection |

> **Note**: Detection only processes frames when `detection_enable` is `True`. The navigation node enables detection when reaching the first waypoint and disables it on mission complete.

## Publishers

| Topic                     | Type              | Description          |
| ------------------------- | ----------------- | -------------------- |
| `/drone1/disease_geotag`  | `GeoPointStamped` | Detection GPS coords |
| `/drone1/detection_image` | `Image`           | Debug visualization  |

## Parameters

| Parameter            | Default          | Description            |
| -------------------- | ---------------- | ---------------------- |
| `hsv_lower`          | `[20, 100, 100]` | Yellow HSV lower bound |
| `hsv_upper`          | `[35, 255, 255]` | Yellow HSV upper bound |
| `min_contour_area`   | `500`            | Minimum pixel area     |
| `duplicate_radius_m` | `2.0`            | Dedup distance         |
| `camera_fov_deg`     | `62.2`           | Camera field of view   |

## Key Functions

### `image_callback(msg: Image)`

Main processing function called for each frame.

```python
def image_callback(self, msg):
    # 1. Convert ROS Image to OpenCV
    cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    # 2. Detect yellow regions
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
    contours, _ = cv2.findContours(mask, ...)

    # 3. Process each contour
    for contour in contours:
        if cv2.contourArea(contour) > self.min_area:
            cx, cy = get_centroid(contour)
            gps = self.pixel_to_gps(cx, cy)
            if not self.is_duplicate(gps):
                self.publish_geotag(gps)
```

### `pixel_to_gps(px, py) -> (lat, lon)`

Ray-casting from pixel to ground GPS.

```python
def pixel_to_gps(self, px, py):
    # 1. Get pixel offset from image center
    dx = px - self.image_width / 2
    dy = py - self.image_height / 2

    # 2. Convert to meters using altitude and FOV
    meters_per_pixel = self.altitude * 2 * tan(fov/2) / image_width
    offset_x = dx * meters_per_pixel
    offset_y = dy * meters_per_pixel

    # 3. Rotate by drone heading (from IMU)
    rotated_x = offset_x * cos(yaw) - offset_y * sin(yaw)
    rotated_y = offset_x * sin(yaw) + offset_y * cos(yaw)

    # 4. Add to current GPS
    lat = current_lat + (rotated_y / 111320)  # meters to degrees
    lon = current_lon + (rotated_x / (111320 * cos(lat)))
    return lat, lon
```

### `is_duplicate(lat, lon) -> bool`

Checks if detection is within radius of previous detections.

```python
def is_duplicate(self, lat, lon):
    for prev_lat, prev_lon in self.detection_history:
        distance = haversine(lat, lon, prev_lat, prev_lon)
        if distance < self.duplicate_radius:
            return True
    return False
```

## Debugging

### Check Detection is Working

```bash
# View detection count
ros2 topic hz /drone1/disease_geotag

# View debug image
ros2 run rqt_image_view rqt_image_view /drone1/detection_image
```

### Common Issues

| Issue                 | Cause               | Debug                         |
| --------------------- | ------------------- | ----------------------------- |
| No detections         | HSV threshold wrong | View mask in debug image      |
| Wrong GPS             | IMU data missing    | Check `/mavros/imu/data`      |
| Many duplicates       | Radius too small    | Increase `duplicate_radius_m` |
| Detections off-target | FOV incorrect       | Calibrate `camera_fov_deg`    |

### CSV Log Format

```csv
timestamp,latitude,longitude,altitude,confidence,pixel_x,pixel_y
1703524800,37.619000,-122.357500,10.0,0.85,320,240
```
