# Detection and Geotag Node - Developer Documentation

## Overview

**File**: `drone1_ws/src/detection_and_geotag/detection_and_geotag/detection_and_geotag_node.py`  
**Package**: `detection_and_geotag`  
**Node Name**: `detection_and_geotag_node`  
**Author**: Shaan Shoukath

## Purpose

Detects yellow disease using HSV color thresholding and computes GPS geotags by projecting pixel coordinates to world coordinates using drone altitude and IMU orientation.

---

## Hardware Preconditions

```
Pi Camera 3 ─── CSI ───► Raspberry Pi 5
Camera pointing straight down
GPS ───► Cube Orange+ (for geotag computation)
```

## ArduPilot Parameters

```
# GPS required for geotagging
GPS_TYPE = 1             # Auto
GPS_GNSS_MODE = 0        # All constellations
```

---

## Key Parameters

```yaml
image_width: 720
image_height: 720
target_fps: 10.0

# Yellow HSV range
yellow_h_min: 15
yellow_h_max: 40
yellow_s_min: 80
yellow_v_min: 80

min_detection_area: 250
gps_dedup_distance_m: 3.0
```

---

## Subscribers

| Topic                            | Type        | Purpose       |
| -------------------------------- | ----------- | ------------- |
| `/camera/image_raw`              | Image       | Camera frames |
| `/mavros/global_position/global` | NavSatFix   | Drone GPS     |
| `/mavros/imu/data`               | Imu         | Orientation   |
| `/mavros/local_position/pose`    | PoseStamped | Altitude      |

## Publishers

| Topic                     | Type            | Purpose       |
| ------------------------- | --------------- | ------------- |
| `/drone1/disease_geotag`  | GeoPointStamped | For telem_tx  |
| `/drone1/detection_debug` | Image           | Visualization |

---

## Detection Pipeline

```
Camera → HSV → Yellow Mask → Morphology → Contours → Validation → Ray-casting → Geotag
```

---

**Last Updated**: January 5, 2026
