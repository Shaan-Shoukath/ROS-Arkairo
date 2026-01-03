# Detection and Geotag Node - Developer Documentation

## Overview

**File**: `drone1_ws/src/detection_and_geotag/detection_and_geotag/detection_and_geotag_node.py`  
**Package**: `detection_and_geotag`  
**Node Name**: `detection_and_geotag_node`  
**Author**: Shaan Shoukath

## Purpose

Detects yellow plant disease using computer vision and computes GPS geotags by projecting pixel coordinates to world coordinates using drone altitude and IMU orientation.

## Features

- HSV color thresholding for yellow disease
- Green vegetation context validation
- Sand/soil exclusion filtering
- Shape-based validation (aspect ratio, compactness)
- GPS geotagging with deduplication
- Severity classification (MILD/MODERATE/SEVERE)
- Ray-casting GPS projection using IMU

## Key Parameters

Located in: `config/detection_params.yaml`

```yaml
# Camera (720p square)
image_width: 720
image_height: 720
target_fps: 10.0 # Frame processing rate

# Yellow disease HSV range
yellow_h_min: 15
yellow_h_max: 40
yellow_s_min: 80
yellow_v_min: 80

# Detection thresholds
min_detection_area: 250 # Min pixels for detection
gps_dedup_distance_m: 3.0 # Deduplication radius
```

## Subscribers

| Topic                            | Type        | Purpose                     |
| -------------------------------- | ----------- | --------------------------- |
| `/camera/image_raw`              | Image       | Camera frames               |
| `/mavros/global_position/global` | NavSatFix   | Drone GPS                   |
| `/mavros/imu/data`               | Imu         | Orientation for ray-casting |
| `/mavros/local_position/pose`    | PoseStamped | Altitude                    |
| `/drone1/detection_enable`       | Bool        | Enable/disable from nav     |

## Publishers

| Topic                     | Type            | Purpose             |
| ------------------------- | --------------- | ------------------- |
| `/drone1/disease_geotag`  | GeoPointStamped | Detection GPS       |
| `/drone1/detection_debug` | Image           | Debug visualization |

## Detection Pipeline

```
Camera Frame → HSV → Yellow/Green/Brown Masks → Morphological Filtering
    → Contour Detection → Validation → Ray-casting GPS → Deduplication → Publish
```

## Severity Classification

| Area (pixels) | Severity |
| ------------- | -------- |
| < 500         | MILD     |
| 500 - 1000    | MODERATE |
| > 1000        | SEVERE   |

---

**Last Updated**: January 2, 2026
