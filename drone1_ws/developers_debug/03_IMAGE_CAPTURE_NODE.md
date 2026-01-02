# Image Capture Node - Developer Documentation

## Overview

**File**: `drone1_ws/src/image_capture/image_capture/image_capture_node.py`  
**Package**: `image_capture`  
**Node Name**: `image_capture_node`  
**Purpose**: Camera abstraction layer for capturing and publishing images

## What This Node Does

Dual-mode camera interface that:

1. **Selects camera source** based on `use_sim` parameter
2. **use_sim=true**: Uses laptop webcam (for SITL testing)
3. **use_sim=false**: Uses Pi Camera 3 Wide via libcamera (for real hardware)
4. **Publishes raw images** to ROS2 topics
5. **Falls back to test images** if camera fails

## Camera Modes (Jan 2026 Update)

| Mode            | Camera Source             | Use Case              |
| --------------- | ------------------------- | --------------------- |
| `use_sim=true`  | Laptop webcam (index 0)   | SITL testing          |
| `use_sim=false` | Pi Camera 3 via libcamera | Real hardware on Pi 5 |

## Core Logic

```python
def _init_camera(self):
    if self.use_sim:
        # SITL: Use laptop webcam
        self.cap = cv2.VideoCapture(self.webcam_index)
    else:
        # Real hardware: Pi Camera via libcamera GStreamer
        pipeline = 'libcamerasrc ! video/x-raw... ! appsink'
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            # Fallback to v4l2
            self.cap = cv2.VideoCapture(self.pi_camera_device)
```

## Publishers

### 1. `/camera/image_raw` (sensor_msgs/Image)

- **Rate**: Configured FPS (default 30Hz)
- **Format**: BGR8 (OpenCV default)
- **Purpose**: Raw camera frames for disease detection
- **QoS**: BEST_EFFORT (prioritize latency over reliability)

### 2. `/camera/camera_info` (sensor_msgs/CameraInfo)

- **Rate**: Synchronized with image
- **Purpose**: Camera intrinsics for geotagging calculations
- **Data**: Focal length, principal point, distortion coefficients

## Parameters

| Parameter                   | Default                | Description                          |
| --------------------------- | ---------------------- | ------------------------------------ |
| **Mode Selection**          |                        |                                      |
| `use_sim`                   | true                   | true=laptop webcam, false=Pi Camera  |
| **Camera Devices**          |                        |                                      |
| `webcam_index`              | 0                      | Laptop webcam index (use_sim=true)   |
| `pi_camera_device`          | '/dev/video0'          | Pi Camera v4l2 fallback              |
| `camera_device`             | '/dev/video0'          | Legacy: general camera device        |
| **Resolution**              |                        |                                      |
| `image_width`               | 1920                   | Frame width (pixels)                 |
| `image_height`              | 1080                   | Frame height (pixels)                |
| `fps`                       | 30.0                   | Capture frame rate                   |
| `frame_id`                  | 'camera_optical_frame' | TF frame name                        |
| **Camera Calibration**      |                        |                                      |
| `camera_matrix.fx`          | 1000.0                 | Focal length X (pixels)              |
| `camera_matrix.fy`          | 1000.0                 | Focal length Y (pixels)              |
| `camera_matrix.cx`          | 960.0                  | Principal point X                    |
| `camera_matrix.cy`          | 540.0                  | Principal point Y                    |
| `distortion_coefficients.*` | 0.0                    | Lens distortion (k1, k2, p1, p2, k3) |

## Key Functions

### `_init_camera()` - Dual Mode Camera Init

### `capture_and_publish()` - Main Loop

```python
def capture_and_publish(self):
    ret, frame = self.cap.read()
    if ret:
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(img_msg)
```

**Built-in Functions**:

- `cap.read()` - Captures single frame, returns (success_bool, frame_array)
- `cv2_to_imgmsg()` - Converts OpenCV image to ROS Image message

### `get_camera_info()` - Calibration Message

```python
def get_camera_info(self) -> CameraInfo:
    info = CameraInfo()
    info.k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]  # 3x3 camera matrix
    info.d = [k1, k2, p1, p2, k3]             # Distortion coefficients
```

**Purpose**: Packages calibration data for 3D geometry calculations

## Package Dependencies

### ROS2 Packages

- **rclpy**: Python client
- **sensor_msgs**: `Image`, `CameraInfo`
- **cv_bridge**: `CvBridge` - OpenCV ↔ ROS message converter

### Python Libraries

- **cv2** (OpenCV): Computer vision library
  - `VideoCapture` - Camera interface
  - `CAP_PROP_*` - Configuration constants
  - `imshow()`, `waitKey()` - Display functions (optional)
- **numpy** (`np`): Array operations
  - Frame data is numpy arrays
  - `np.uint8` - Image data type

## Camera Calibration

**Why calibrate?**  
Camera lenses distort images (barrel/pincushion distortion). Calibration parameters correct this for accurate geotagging.

**Calibration Process** (external tool):

1. Print checkerboard pattern
2. Capture 20+ images from different angles
3. Run OpenCV calibration: `cv2.calibrateCamera()`
4. Extract fx, fy, cx, cy, k1-k3 parameters
5. Update node parameters

**What each parameter means**:

- **fx, fy**: Focal length in pixels (lens zoom)
- **cx, cy**: Image center (optical axis intersection)
- **k1, k2, k3**: Radial distortion coefficients
- **p1, p2**: Tangential distortion coefficients

## Error Handling

**Camera not found**: Logs error and exits  
**Frame read failure**: Logs warning and continues (dropped frame)  
**USB disconnection**: Attempts reconnection on next read

## Testing Checklist

- [ ] Camera device detected at specified path
- [ ] Images published at correct FPS
- [ ] Image dimensions match configuration
- [ ] CameraInfo published with every image
- [ ] Frames appear correct (no extreme distortion)

## Common Issues

**Issue**: No camera found  
**Solution**: Check device path with `ls /dev/video*`

**Issue**: Low FPS  
**Solution**: Check USB bandwidth, reduce resolution

**Issue**: Black images  
**Solution**: Check camera permissions: `sudo chmod 666 /dev/video0`

## Simulation Testing

### Without Physical Camera

For SITL testing without a USB camera, you can use:

**Option 1: Simulated Camera (ROS2 image publisher)**

```bash
# Use a test video or image sequence
ros2 run image_tools cam2image --ros-args -p reliable:=false
```

**Option 2: Gazebo Simulation Camera**
If using Gazebo with ArduPilot SITL, the simulated camera publishes to the same topic.

**Option 3: Video File Playback**
Modify the node to read from a video file instead of device:

```python
self.cap = cv2.VideoCapture('test_video.mp4')  # Instead of /dev/video0
```

### Verify Camera Output

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/camera_info --once
```

---

**Last Updated**: January 1, 2026  
**Maintainer**: Shaan Shoukath  
**⚠️ Note**: use_sim parameter controls camera source selection
