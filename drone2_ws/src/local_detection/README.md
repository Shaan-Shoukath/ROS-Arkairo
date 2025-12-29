# Local Detection Node

**On-Site Disease Confirmation**

## What It Does

Activates when Drone-2 arrives at a target location. Processes camera frames to re-confirm disease presence locally, filtering false positives from Drone-1's high-altitude survey.

## Logic Flow

```
Arrival Status = True
        │
        ▼
Activate Detection Mode
        │
        ▼
For Each Camera Frame:
├── Apply HSV threshold (same as Drone-1)
├── Find contours
├── Get largest bounding box
├── If detection found:
│   ├── Increment consecutive_count
│   ├── Publish bounding box
│   └── If consecutive_count >= required:
│       └── Publish detection_status = True
│       └── Deactivate
└── Else:
    └── Reset consecutive_count
        │
        ▼ [timeout]
Publish detection_status = False → Skip target
```

## Subscribers

| Topic                    | Type    | Callback             | Description      |
| ------------------------ | ------- | -------------------- | ---------------- |
| `/drone2/arrival_status` | `Bool`  | `arrival_callback()` | Trigger to start |
| `/camera/image_raw`      | `Image` | `image_callback()`   | Camera frames    |

## Publishers

| Topic                            | Type          | Description                |
| -------------------------------- | ------------- | -------------------------- |
| `/drone2/local_detection_status` | `Bool`        | Confirmed yes/no           |
| `/drone2/detection_bbox`         | `Detection2D` | Bounding box for centering |

## Parameters

| Parameter               | Default          | Description              |
| ----------------------- | ---------------- | ------------------------ |
| `hsv_lower`             | `[20, 100, 100]` | Yellow lower bound       |
| `hsv_upper`             | `[35, 255, 255]` | Yellow upper bound       |
| `min_contour_area`      | `1000`           | Minimum detection size   |
| `consecutive_required`  | `3`              | Frames needed to confirm |
| `detection_timeout_sec` | `10.0`           | Max time to detect       |

## Key Functions

### `arrival_callback(msg: Bool)`

Activates detection when arrived at target.

```python
def arrival_callback(self, msg):
    if msg.data:
        self.detecting = True
        self.consecutive_detections = 0
        self.detection_start_time = self.get_clock().now()
        self.get_logger().info('Arrived - starting local detection')
```

### `image_callback(msg: Image)`

Processes frames during active detection.

```python
def image_callback(self, msg):
    if not self.detecting:
        return

    # Check timeout
    if self._check_timeout():
        self._publish_status(False)
        self.detecting = False
        return

    cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    bbox = self._detect_disease(cv_image)

    if bbox:
        self.consecutive_detections += 1
        self._publish_bbox(bbox, msg.header)

        if self.consecutive_detections >= self.consec_required:
            self._publish_status(True)
            self.detecting = False
            self.get_logger().info('Disease confirmed locally!')
    else:
        self.consecutive_detections = 0  # Reset on miss
```

### `_detect_disease(cv_image) -> Optional[BoundingBox]`

HSV detection matching Drone-1 algorithm.

```python
def _detect_disease(self, cv_image):
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None

    # Get largest contour
    largest = max(contours, key=cv2.contourArea)

    if cv2.contourArea(largest) < self.min_area:
        return None

    x, y, w, h = cv2.boundingRect(largest)
    return BoundingBox(x=x, y=y, width=w, height=h)
```

### `_publish_bbox(bbox, header)`

Publishes Detection2D for centering controller.

```python
def _publish_bbox(self, bbox, header):
    msg = Detection2D()
    msg.header = header
    msg.bbox.center.position.x = bbox.x + bbox.width / 2
    msg.bbox.center.position.y = bbox.y + bbox.height / 2
    msg.bbox.size_x = bbox.width
    msg.bbox.size_y = bbox.height
    self.bbox_pub.publish(msg)
```

## Debugging

### Check Detection is Running

```bash
# Monitor status
ros2 topic echo /drone2/local_detection_status

# View bounding boxes
ros2 topic echo /drone2/detection_bbox
```

### Common Issues

| Issue             | Cause                          | Debug                    |
| ----------------- | ------------------------------ | ------------------------ |
| Never confirms    | Threshold mismatch             | Match Drone-1 HSV params |
| Too many confirms | `consecutive_required` too low | Increase to 5            |
| Times out         | Target not visible             | Check camera angle       |
| False skips       | Area threshold high            | Lower `min_contour_area` |

## Multi-Frame Confirmation Logic

Why require multiple consecutive detections?

- Single frame can have noise/glare
- Movement causes blur
- Confirms stable detection before spray
- Reduces false positive rate
