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
use_sim: true # true = laptop webcam, false = Pi Camera
webcam_index: 0 # Laptop webcam index for SITL
pi_camera_device: "/dev/video0" # Pi Camera via v4l2
image_width: 1920
image_height: 1080
fps: 30.0
```

## Subscribers

| Topic                       | Type  | Purpose                          |
| --------------------------- | ----- | -------------------------------- |
| `/camera/inject_test_image` | Image | Injected test images (SITL only) |

## Publishers

| Topic                 | Type       | Purpose            |
| --------------------- | ---------- | ------------------ |
| `/camera/image_raw`   | Image      | Raw camera frames  |
| `/camera/camera_info` | CameraInfo | Camera calibration |

## Test Image Injection (SITL Only)

When `use_sim=true`, you can inject test images during flight:

```bash
# Inject a synthetic yellow test image to trigger detection
ros2 run image_capture publish_test_image

# Inject a custom image file
ros2 run image_capture publish_test_image --image /path/to/test.png
```

The injected image will be published once on `/camera/image_raw`, triggering the detection pipeline.

## Hardware Setup

- Pi Camera v1/v2/v3 connected via CSI
- Mounted pointing straight down
- FOV: 62° horizontal

---

**Last Updated**: January 6, 2026
