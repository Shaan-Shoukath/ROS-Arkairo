# Centering Controller Node

**PID Visual Servoing for Spray Alignment**

## What It Does

Uses PID control to precisely center the spray nozzle over the detected disease. Subscribes to bounding box from local detection and publishes velocity commands to align before spraying.

## Logic Flow

```
Detection Status = True (from local_detection)
        │
        ▼
Activate Centering Mode
        │
        ▼
For Each Bounding Box:
├── Calculate pixel error (bbox center vs image center)
├── Apply PID control
│   ├── error_x = bbox_center_x - image_center_x
│   ├── error_y = bbox_center_y - image_center_y
│   ├── integral += error * dt
│   └── derivative = (error - prev_error) / dt
├── Calculate velocity commands
│   ├── vx = Kp*error_x + Ki*integral_x + Kd*derivative_x
│   └── vy = Kp*error_y + Ki*integral_y + Kd*derivative_y
├── Clamp velocities to max
├── Publish velocity command
└── If error < threshold:
    ├── Stop motion
    └── Publish spray_ready = True
```

## Subscribers

| Topic                            | Type          | Callback            | Description        |
| -------------------------------- | ------------- | ------------------- | ------------------ |
| `/drone2/local_detection_status` | `Bool`        | `status_callback()` | Activation trigger |
| `/drone2/detection_bbox`         | `Detection2D` | `bbox_callback()`   | Target position    |

## Publishers

| Topic                               | Type           | Description        |
| ----------------------------------- | -------------- | ------------------ |
| `/mavros/setpoint_velocity/cmd_vel` | `TwistStamped` | Velocity commands  |
| `/drone2/spray_ready`               | `Bool`         | Centered and ready |

## Parameters

| Parameter               | Default  | Description           |
| ----------------------- | -------- | --------------------- |
| `kp`                    | `0.002`  | Proportional gain     |
| `ki`                    | `0.0001` | Integral gain         |
| `kd`                    | `0.001`  | Derivative gain       |
| `max_velocity_mps`      | `0.5`    | Max output velocity   |
| `centered_threshold_px` | `20`     | Pixel error tolerance |
| `image_width`           | `640`    | Camera width          |
| `image_height`          | `480`    | Camera height         |
| `centering_timeout_sec` | `15.0`   | Max centering time    |

## Key Functions

### `status_callback(msg: Bool)`

Activates centering when detection confirmed.

```python
def status_callback(self, msg):
    if msg.data:
        self.centering_active = True
        self._reset_pid()
        self.centering_start = self.get_clock().now()
        self.get_logger().info('Detection confirmed - starting centering')
```

### `bbox_callback(msg: Detection2D)`

Main control callback for each bounding box.

```python
def bbox_callback(self, msg):
    if not self.centering_active:
        return

    self.last_bbox = msg

    # Calculate error
    center_x = self.image_width / 2
    center_y = self.image_height / 2
    error_x = msg.bbox.center.position.x - center_x
    error_y = msg.bbox.center.position.y - center_y

    # Check if centered
    if abs(error_x) < self.threshold and abs(error_y) < self.threshold:
        self._stop_motion()
        self._publish_ready(True)
        self.centering_active = False
        return

    # PID control
    vx, vy = self._calculate_pid(error_x, error_y)
    self._publish_velocity(vx, vy)
```

### `_calculate_pid(error_x, error_y) -> (vx, vy)`

PID controller implementation.

```python
def _calculate_pid(self, error_x, error_y):
    now = self.get_clock().now().nanoseconds / 1e9
    dt = now - self.last_time if self.last_time else 0.1
    self.last_time = now

    # X-axis PID
    self.integral_x += error_x * dt
    derivative_x = (error_x - self.prev_error_x) / dt if dt > 0 else 0
    vx = self.kp * error_x + self.ki * self.integral_x + self.kd * derivative_x
    self.prev_error_x = error_x

    # Y-axis PID
    self.integral_y += error_y * dt
    derivative_y = (error_y - self.prev_error_y) / dt if dt > 0 else 0
    vy = self.kp * error_y + self.ki * self.integral_y + self.kd * derivative_y
    self.prev_error_y = error_y

    # Clamp to max velocity
    vx = max(-self.max_vel, min(self.max_vel, vx))
    vy = max(-self.max_vel, min(self.max_vel, vy))

    return vx, vy
```

### `_publish_velocity(vx, vy)`

Sends velocity commands to MAVROS.

```python
def _publish_velocity(self, vx, vy):
    msg = TwistStamped()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.twist.linear.x = vx  # Forward/backward
    msg.twist.linear.y = vy  # Left/right
    msg.twist.linear.z = 0.0 # Maintain altitude
    self.vel_pub.publish(msg)
```

## Debugging

### Monitor Centering

```bash
# Check velocity commands
ros2 topic echo /mavros/setpoint_velocity/cmd_vel

# Check if ready
ros2 topic echo /drone2/spray_ready
```

### PID Tuning

| Symptom        | Adjustment               |
| -------------- | ------------------------ |
| Oscillating    | Decrease Kp, increase Kd |
| Slow to center | Increase Kp              |
| Never settles  | Decrease Ki              |
| Overshoots     | Increase Kd              |

### Common Issues

| Issue         | Cause                | Debug               |
| ------------- | -------------------- | ------------------- |
| Never centers | Threshold too tight  | Increase to 30-50px |
| Oscillates    | Kp too high          | Reduce by 50%       |
| Drifts away   | Camera axis inverted | Flip sign of vx/vy  |
| Times out     | Target lost          | Check detection     |
