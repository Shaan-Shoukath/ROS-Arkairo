# Image Capture Node - Developer Documentation

## Overview

**File**: `drone1_ws/src/image_capture/image_capture/image_capture_node.py`  
**Package**: `image_capture`  
**Node Name**: `image_capture_node`  
**Purpose**: Camera abstraction layer for capturing and publishing images

## What This Node Does

Simple camera interface that:

1. **Opens USB camera device** (e.g., `/dev/video0`)
2. **Captures frames continuously** at specified FPS
3. **Publishes raw images** to ROS2 topics
4. **Provides camera calibration info** for downstream processing

## Core Logic

```python
while True:
    ret, frame = camera.read()  # Capture frame
    if ret:
        publish_image(frame)    # Convert to ROS msg and publish
        publish_camera_info()   # Send calibration data
```

**Design**: Simple blocking loop, no complex state management needed.

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
| `camera_device`             | '/dev/video0'          | Camera device path                   |
| `use_usb_camera`            | True                   | Enable USB camera capture            |
| `image_width`               | 1920                   | Frame width (pixels)                 |
| `image_height`              | 1080                   | Frame height (pixels)                |
| `fps`                       | 30.0                   | Capture frame rate                   |
| `frame_id`                  | 'camera_optical_frame' | TF frame name                        |
| `camera_matrix.fx`          | 1000.0                 | Focal length X (pixels)              |
| `camera_matrix.fy`          | 1000.0                 | Focal length Y (pixels)              |
| `camera_matrix.cx`          | 960.0                  | Principal point X                    |
| `camera_matrix.cy`          | 540.0                  | Principal point Y                    |
| `distortion_coefficients.*` | 0.0                    | Lens distortion (k1, k2, p1, p2, k3) |

## Key Functions

### `_init_camera()` - Camera Initialization

```python
def _init_camera(self):
    self.cap = cv2.VideoCapture(self.camera_device)
    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
    self.cap.set(cv2.CAP_PROP_FPS, self.fps)
```

**Built-in Functions**:

- `cv2.VideoCapture()` - Opens camera device
- `cap.set()` - Configures camera properties
- `CAP_PROP_*` - OpenCV camera property constants

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

---

**Last Updated**: December 30, 2025
