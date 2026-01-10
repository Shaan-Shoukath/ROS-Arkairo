# Image Capture Node - Developer Documentation

## Overview

| Property        | Value                                                             |
| --------------- | ----------------------------------------------------------------- |
| **File**        | `drone1_ws/src/image_capture/image_capture/image_capture_node.py` |
| **Package**     | `image_capture`                                                   |
| **Node Name**   | `image_capture_node`                                              |
| **Config File** | `image_capture/config/camera_params.yaml`                         |
| **Maintainer**  | Shaan Shoukath                                                    |

## Purpose

Captures images from Pi Camera (hardware) or webcam (SITL) and publishes to ROS2 for detection processing.

---

## Configuration YAML → Code Mapping

```yaml
# camera_params.yaml                      # Python code usage
use_sim: true                     →  self.use_sim (true=webcam, false=Pi Camera)
webcam_index: 0                   →  cv2.VideoCapture(webcam_index)
pi_camera_device: "/dev/video0"   →  GStreamer pipeline source
image_width: 1280                 →  self.image_width, cap.set(CAP_PROP_FRAME_WIDTH)
image_height: 720                 →  self.image_height, cap.set(CAP_PROP_FRAME_HEIGHT)
fps: 30.0                         →  Timer rate for capture_callback
```

---

## Key Functions

| Function                    | Purpose                                     |
| --------------------------- | ------------------------------------------- |
| `__init__`                  | Declare parameters, call \_init_camera      |
| `_init_camera`              | Open webcam (SITL) or Pi Camera (GStreamer) |
| `capture_callback`          | Read frame, convert to ROS Image, publish   |
| `inject_callback`           | Receive injected test images (SITL)         |
| `_build_gstreamer_pipeline` | Create GStreamer string for Pi Camera       |

---

## Camera Modes

| Mode     | `use_sim` | Camera Source            |
| -------- | --------- | ------------------------ |
| Hardware | `false`   | Pi Camera via GStreamer  |
| SITL     | `true`    | Laptop webcam via OpenCV |

---

## Subscribers

| Topic                       | Type  | Callback          | Purpose                     |
| --------------------------- | ----- | ----------------- | --------------------------- |
| `/camera/inject_test_image` | Image | `inject_callback` | Injected test images (SITL) |

## Publishers

| Topic                 | Type       | Purpose            |
| --------------------- | ---------- | ------------------ |
| `/camera/image_raw`   | Image      | Raw camera frames  |
| `/camera/camera_info` | CameraInfo | Camera calibration |

---

## Test Image Injection (SITL Only)

When `use_sim=true`, you can inject test images:

```bash
# Inject synthetic yellow test image
ros2 run image_capture publish_test_image

# Inject custom image file
ros2 run image_capture publish_test_image --image /path/to/test.png
```

---

## Launch Commands

```bash
# With config file
ros2 run image_capture image_capture_node --ros-args \
  --params-file ~/Documents/ROS-Arkairo/drone1_ws/src/image_capture/config/camera_params.yaml

# Override for SITL webcam
ros2 run image_capture image_capture_node --ros-args \
  -p use_sim:=true \
  -p image_width:=640 \
  -p image_height:=480
```

---

## Hardware Setup

- Pi Camera v1/v2/v3 connected via CSI
- Mounted pointing straight down
- FOV: 62° horizontal

---

**Last Updated**: January 10, 2026
