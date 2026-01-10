# Detection and Geotag Node - Developer Documentation

## Overview

| Property        | Value                                                                                  |
| --------------- | -------------------------------------------------------------------------------------- |
| **File**        | `drone1_ws/src/detection_and_geotag/detection_and_geotag/detection_and_geotag_node.py` |
| **Package**     | `detection_and_geotag`                                                                 |
| **Node Name**   | `detection_and_geotag_node`                                                            |
| **Config File** | `detection_and_geotag/config/detection_params.yaml`                                    |
| **Maintainer**  | Shaan Shoukath                                                                         |

## Purpose

Detects yellow disease using HSV color thresholding and computes GPS geotags by ray-casting from pixel coordinates to world coordinates.

---

## State Variables

| Variable                 | Type      | Purpose                                        |
| ------------------------ | --------- | ---------------------------------------------- |
| `self.current_gps`       | NavSatFix | Latest drone GPS (needed for geotagging)       |
| `self.current_imu`       | Imu       | Drone orientation (roll/pitch for ray-casting) |
| `self.current_altitude`  | float     | Height above ground                            |
| `self.detection_enabled` | bool      | Whether to process images (controlled by nav)  |
| `self.detection_counter` | int       | Total detections logged                        |
| `self.recent_geotags`    | List      | GPS positions for deduplication                |
| `self.last_process_time` | float     | For FPS limiting                               |
| `self.yellow_min/max`    | np.array  | HSV thresholds for yellow detection            |
| `self.green_min/max`     | np.array  | HSV thresholds for plant context               |
| `self.show_gui`          | bool      | Whether to show OpenCV visualization           |

---

## Configuration YAML → Code Mapping

```yaml
# detection_params.yaml                   # Where Used
image_width: 720                  →  Camera intrinsics for ray-casting
image_height: 720                 →  Camera intrinsics
target_fps: 10.0                  →  Frame rate limiter in image_callback
min_detection_area: 250           →  Filter small contours in _detect_yellow_disease
gps_dedup_distance_m: 3.0         →  Ignore detections within 3m of previous

# HSV Color Ranges
yellow_h_min: 15                  →  self.yellow_min[0] (Hue lower bound)
yellow_h_max: 40                  →  self.yellow_max[0] (Hue upper bound)
yellow_s_min: 80                  →  self.yellow_min[1] (Saturation)
yellow_v_min: 80                  →  self.yellow_min[2] (Value/brightness)

# Debug
publish_debug: true               →  Whether to publish annotated image
show_gui: false                   →  OpenCV window for SITL testing
log_to_csv: true                  →  Log detections to CSV file
```

---

## Key Functions and Why They Exist

### Callbacks (React to external events)

| Function                    | Why It Exists                          | Variables Used      |
| --------------------------- | -------------------------------------- | ------------------- |
| `detection_enable_callback` | Navigation enables/disables processing | `detection_enabled` |
| `image_callback`            | Main entry - process each camera frame | All state variables |
| `gps_callback`              | Update current GPS for geotagging      | `current_gps`       |
| `imu_callback`              | Update orientation for ray-casting     | `current_imu`       |
| `pose_callback`             | Update altitude for geotagging         | `current_altitude`  |

### Detection Functions

| Function                 | Why It Exists                                    | Variables Used                                |
| ------------------------ | ------------------------------------------------ | --------------------------------------------- |
| `_detect_yellow_disease` | Core detection: HSV → mask → contours → validate | `yellow_min/max`, `green_min/max`, `min_area` |
| `_is_in_green_context`   | Ensure yellow is near vegetation (not sand)      | `green_min/max`                               |
| `_validate_shape`        | Filter by aspect ratio, solidity                 | Contour properties                            |
| `_is_duplicate`          | Skip if already detected nearby                  | `recent_geotags`, `dedup_distance`            |

### Geotagging Functions

| Function          | Why It Exists                          | Variables Used                                   |
| ----------------- | -------------------------------------- | ------------------------------------------------ |
| `_compute_geotag` | Convert pixel (x,y) → world GPS        | `current_gps`, `current_altitude`, `current_imu` |
| `_pixel_to_ray`   | Camera intrinsics: pixel → unit vector | `image_width`, `image_height`, FOV               |
| `_ray_to_ground`  | Intersect ray with ground plane        | altitude, orientation                            |
| `_local_to_gps`   | ENU offset → GPS coordinates           | Earth radius, home GPS                           |

### Output Functions

| Function                  | Why It Exists                    | Variables Used          |
| ------------------------- | -------------------------------- | ----------------------- |
| `_publish_geotag`         | Send detection to telem_tx       | GeoPointStamped message |
| `_draw_detections`        | Annotate frame with circles/text | Detection list          |
| `_publish_debug_image`    | Publish annotated frame          | ROS Image               |
| `_show_gui_visualization` | OpenCV window for SITL debugging | `show_gui`              |
| `_log_to_csv`             | Write detection to CSV           | `csv_path`              |

---

## Detection Pipeline

```
Navigation Enable → Check GPS exists → FPS limit check
         ↓
   Camera Frame
         ↓
   BGR → HSV color space
         ↓
   Yellow mask (inRange)
         ↓
   Morphology cleanup (open/close)
         ↓
   Green context filter (require near plants)
         ↓
   Find contours
         ↓
   For each contour:
      → Area check (> min_area)
      → Shape validation
      → Deduplication (> 3m from previous)
      → Ray-cast to GPS
      → Publish geotag
```

---

## Why Detection is Gated

Detection only runs when:

1. `detection_enabled == True` (set by navigation during NAVIGATE state)
2. `current_gps != None` (need GPS for geotagging)
3. Frame rate limit not exceeded (prevent CPU overload)

This prevents:

- Processing during takeoff/landing
- Geotagging without GPS lock
- CPU overload from high frame rates

---

## Subscribers

| Topic                            | Type        | Callback                    | Purpose       |
| -------------------------------- | ----------- | --------------------------- | ------------- |
| `/camera/image_raw`              | Image       | `image_callback`            | Camera frames |
| `/mavros/global_position/global` | NavSatFix   | `gps_callback`              | Drone GPS     |
| `/mavros/imu/data`               | Imu         | `imu_callback`              | Orientation   |
| `/mavros/local_position/pose`    | PoseStamped | `pose_callback`             | Altitude      |
| `/drone1/detection_enable`       | Bool        | `detection_enable_callback` | On/off        |

## Publishers

| Topic                     | Type            | Purpose       |
| ------------------------- | --------------- | ------------- |
| `/drone1/disease_geotag`  | GeoPointStamped | For telem_tx  |
| `/drone1/detection_debug` | Image           | Visualization |

---

## Launch Command

```bash
ros2 run detection_and_geotag detection_and_geotag_node --ros-args \
  --params-file ~/Documents/ROS-Arkairo/drone1_ws/src/detection_and_geotag/config/detection_params.yaml

# With GUI for SITL
ros2 run detection_and_geotag detection_and_geotag_node --ros-args -p show_gui:=true
```

---

**Last Updated**: January 10, 2026
