# Image Capture Node - Developer Documentation

## Overview

**File**: `drone1_ws/src/image_capture/image_capture/image_capture_node.py`  
**Package**: `image_capture`  
**Node Name**: `image_capture_node`  
**Author**: Shaan Shoukath

## Purpose

Captures images from Pi Camera and publishes to ROS2 for detection processing.

## Key Parameters

Located in: `config/camera_params.yaml`

```yaml
# Camera resolution (720p square recommended)
image_width: 720
image_height: 720
fps: 30

# Camera device
device_id: 0
```

## Subscribers

None (captures directly from camera hardware)

## Publishers

| Topic                 | Type       | Purpose            |
| --------------------- | ---------- | ------------------ |
| `/camera/image_raw`   | Image      | Raw camera frames  |
| `/camera/camera_info` | CameraInfo | Camera calibration |

## Hardware Setup

- Pi Camera v1/v2/v3 connected via CSI
- Mounted pointing straight down
- FOV: 62° horizontal

---

**Last Updated**: January 2, 2026
