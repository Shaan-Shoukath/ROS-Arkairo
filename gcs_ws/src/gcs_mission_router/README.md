# GCS Mission Router Node

**Mission Dispatch to Drone-2**

## What It Does

Manages the queue of validated targets and dispatches them to Drone-2 when available. Monitors Drone-2 status and implements rate limiting for safety.

## Logic Flow

```
Validated Target from Receiver
        │
        ▼
Add to Pending Queue
        │
        ▼
Dispatch Loop (1Hz):
├── Check Drone-2 Status
│   ├── IDLE? ─────────────────────────┐
│   ├── NAVIGATING/SPRAYING? ──────────┤ Wait
│   └── OFFLINE? ──────────────────────┘
        │                              │
        ▼ [IDLE]                       │
Check Rate Limit (hourly max)          │
        │                              │
        ▼ [under limit]                │
Pop Target from Queue                  │
        │                              │
        ▼                              │
Publish to /drone2/target_geotag       │
        │                              │
        ▼                              │
Increment Hourly Counter               │
```

## Subscribers

| Topic                   | Type              | Callback            | Description   |
| ----------------------- | ----------------- | ------------------- | ------------- |
| `/gcs/validated_target` | `GeoPointStamped` | `target_callback()` | From receiver |
| `/drone2/status`        | `String`          | `status_callback()` | D2 state      |

## Publishers

| Topic                   | Type              | Description   |
| ----------------------- | ----------------- | ------------- |
| `/drone2/target_geotag` | `GeoPointStamped` | Target for D2 |
| `/drone2/mission_start` | `Bool`            | Start trigger |
| `/gcs/queue_size`       | `Int32`           | Pending count |

## Parameters

| Parameter          | Default | Description         |
| ------------------ | ------- | ------------------- |
| `auto_dispatch`    | `true`  | Auto-send to D2     |
| `max_queue_size`   | `100`   | Max pending targets |
| `hourly_limit`     | `20`    | Max dispatches/hour |
| `dispatch_rate_hz` | `1.0`   | Check frequency     |

## Key Functions

### `target_callback(msg: GeoPointStamped)`

Queues incoming validated targets.

```python
def target_callback(self, msg):
    if len(self.pending_queue) >= self.max_queue:
        self.get_logger().warn('Queue full - dropping oldest')
        self.pending_queue.popleft()

    self.pending_queue.append(msg)
    self.get_logger().info(f'Queued target, pending: {len(self.pending_queue)}')
```

### `status_callback(msg: String)`

Tracks Drone-2 operational status.

```python
def status_callback(self, msg):
    self.drone2_status = msg.data

    # Log status changes
    if msg.data != self.prev_status:
        self.get_logger().info(f'Drone-2 status: {msg.data}')
        self.prev_status = msg.data
```

### `dispatch_loop()`

Main dispatch logic executed at dispatch_rate.

```python
def dispatch_loop(self):
    if not self.auto_dispatch:
        return

    if not self._can_dispatch():
        return

    if not self.pending_queue:
        return

    target = self.pending_queue.popleft()
    self._dispatch_target(target)
```

### `_can_dispatch() -> bool`

Checks all dispatch conditions.

```python
def _can_dispatch(self):
    # Drone-2 must be idle
    if self.drone2_status != 'IDLE':
        return False

    # Check hourly rate limit
    if not self._update_hourly_counter():
        self.get_logger().warn('Hourly limit reached')
        return False

    return True
```

### `_dispatch_target(target)`

Sends target to Drone-2.

```python
def _dispatch_target(self, target):
    # Publish target
    self.target_pub.publish(target)

    # Send start trigger
    start_msg = Bool()
    start_msg.data = True
    self.start_pub.publish(start_msg)

    self.dispatched_count += 1
    self.get_logger().info(
        f'Dispatched target to Drone-2 (total: {self.dispatched_count})'
    )
```

### `_update_hourly_counter() -> bool`

Rate limiting implementation.

```python
def _update_hourly_counter(self):
    now = time.time()
    hour_ago = now - 3600

    # Remove old entries
    self.dispatch_times = [t for t in self.dispatch_times if t > hour_ago]

    # Check limit
    if len(self.dispatch_times) >= self.hourly_limit:
        return False

    self.dispatch_times.append(now)
    return True
```

## Debugging

### Monitor Queue

```bash
# Queue size
ros2 topic echo /gcs/queue_size

# Dispatched targets
ros2 topic echo /drone2/target_geotag
```

### Check Drone-2 Status

```bash
ros2 topic echo /drone2/status
```

### Common Issues

| Issue           | Cause            | Debug                  |
| --------------- | ---------------- | ---------------------- |
| Not dispatching | D2 not IDLE      | Check status topic     |
| Queue growing   | D2 offline       | Check D2 nodes         |
| Rate limited    | Too many targets | Wait or increase limit |
