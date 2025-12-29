# Image Capture Node

**Camera Abstraction Layer**

## What It Does

Provides a unified camera interface for the survey drone. Captures frames from USB camera or generates test images for simulation.

## Logic Flow

```
Node Initialization
        │
        ├── Real Mode: OpenCV VideoCapture
        └── Test Mode: Generate colored frames
        │
        ▼
Capture Loop (at frame_rate):
├── Capture frame
├── Convert to ROS Image
├── Publish to /camera/image_raw
└── Publish CameraInfo
```

## Publishers

| Topic                 | Type         | Description       |
| --------------------- | ------------ | ----------------- |
| `/camera/image_raw`   | `Image`      | Raw camera frames |
| `/camera/camera_info` | `CameraInfo` | Camera intrinsics |

## Parameters

| Parameter        | Default       | Description     |
| ---------------- | ------------- | --------------- |
| `camera_device`  | `/dev/video0` | Camera path     |
| `frame_rate`     | `30.0`        | Capture FPS     |
| `image_width`    | `640`         | Frame width     |
| `image_height`   | `480`         | Frame height    |
| `use_test_image` | `false`       | Simulation mode |
| `camera_fov_deg` | `62.2`        | Field of view   |

## Key Functions

### `__init__()`

Initializes camera based on mode.

```python
def __init__(self):
    # ... parameter setup ...

    if self.use_test:
        self.get_logger().info('Using test image mode')
        self._init_test_mode()
    else:
        self._init_camera()

    # Capture timer
    self.timer = self.create_timer(1.0/self.frame_rate, self.capture_callback)
```

### `_init_camera()`

Opens hardware camera.

```python
def _init_camera(self):
    self.cap = cv2.VideoCapture(self.camera_device)

    if not self.cap.isOpened():
        self.get_logger().error(f'Cannot open camera: {self.camera_device}')
        return

    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

    self.get_logger().info(f'Camera opened: {self.camera_device}')
```

### `capture_callback()`

Captures and publishes frames.

```python
def capture_callback(self):
    if self.use_test:
        frame = self._generate_test_frame()
    else:
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

    # Convert to ROS message
    msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = 'camera_optical_frame'

    self.image_pub.publish(msg)
    self._publish_camera_info()
```

### `_generate_test_frame()`

Creates synthetic test images.

```python
def _generate_test_frame(self):
    # Create base image
    frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
    frame[:] = (34, 139, 34)  # Green background (field)

    # Add random yellow spots (simulated disease)
    if random.random() < 0.3:  # 30% chance
        x = random.randint(100, self.width - 100)
        y = random.randint(100, self.height - 100)
        cv2.circle(frame, (x, y), 30, (0, 255, 255), -1)  # Yellow

    return frame
```

### `_publish_camera_info()`

Publishes camera intrinsics.

```python
def _publish_camera_info(self):
    msg = CameraInfo()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = 'camera_optical_frame'
    msg.width = self.width
    msg.height = self.height

    # Pinhole camera model
    fx = fy = self.width / (2 * tan(radians(self.fov) / 2))
    cx, cy = self.width / 2, self.height / 2

    msg.k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    msg.p = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]

    self.camera_info_pub.publish(msg)
```

## Debugging

### Check Frame Rate

```bash
ros2 topic hz /camera/image_raw
```

### View Camera Feed

```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

### Common Issues

| Issue            | Cause                  | Debug                  |
| ---------------- | ---------------------- | ---------------------- |
| No frames        | Camera not connected   | Check `ls /dev/video*` |
| Wrong resolution | Camera doesn't support | Try standard (640x480) |
| Low FPS          | USB bandwidth          | Use USB 3.0 port       |
| Black frames     | Exposure issue         | Check camera settings  |
