# D1 to GCS Uplink Node

**Reliable Geotag Transmission to Ground Control**

## What It Does

Bridges disease detections from Drone-1 to the Ground Control Station via telemetry. Implements queuing, rate limiting, and retry logic for reliable transmission.

## Logic Flow

```
Disease Geotag from Detection Node
        │
        ▼
Add to Transmission Queue
        │
        ▼
Process Queue (rate limited):
├── Check queue not empty
├── Pop oldest geotag
├── Publish to GCS topic
├── Update statistics
└── Publish heartbeat
        │
        ▼
GCS receives via telemetry link
```

## Subscribers

| Topic                    | Type              | Callback            | Description    |
| ------------------------ | ----------------- | ------------------- | -------------- |
| `/drone1/disease_geotag` | `GeoPointStamped` | `geotag_callback()` | From detection |

## Publishers

| Topic                      | Type              | Description |
| -------------------------- | ----------------- | ----------- |
| `/gcs/target_report`       | `GeoPointStamped` | To GCS      |
| `/drone1/uplink_heartbeat` | `Bool`            | Link status |
| `/drone1/uplink_stats`     | `String`          | Queue stats |

## Parameters

| Parameter           | Default | Description                |
| ------------------- | ------- | -------------------------- |
| `max_queue_size`    | `50`    | Max pending transmissions  |
| `transmit_rate_hz`  | `2.0`   | Max send rate              |
| `heartbeat_rate_hz` | `1.0`   | Heartbeat frequency        |
| `enable_retry`      | `true`  | Retry failed transmissions |
| `max_retries`       | `3`     | Retry attempts             |

## Key Functions

### `geotag_callback(msg: GeoPointStamped)`

Queues incoming detections.

```python
def geotag_callback(self, msg):
    if len(self.tx_queue) >= self.max_queue:
        self.get_logger().warn('Queue full - dropping oldest')
        self.tx_queue.popleft()
        self.dropped_count += 1

    self.tx_queue.append(msg)
    self.received_count += 1
```

### `process_queue()`

Rate-limited transmission loop.

```python
def process_queue(self):
    if not self.tx_queue:
        return

    geotag = self.tx_queue.popleft()

    try:
        self.gcs_pub.publish(geotag)
        self.transmitted_count += 1

        self.get_logger().info(
            f'Sent geotag: ({geotag.position.latitude:.6f}, '
            f'{geotag.position.longitude:.6f})'
        )
    except Exception as e:
        self.get_logger().error(f'Transmission failed: {e}')

        if self.enable_retry:
            self._queue_retry(geotag)
```

### `_queue_retry(geotag)`

Handles failed transmission retry.

```python
def _queue_retry(self, geotag):
    retry_count = getattr(geotag, 'retry_count', 0)

    if retry_count < self.max_retries:
        geotag.retry_count = retry_count + 1
        self.tx_queue.appendleft(geotag)  # Priority retry
        self.get_logger().info(f'Retry queued ({retry_count + 1}/{self.max_retries})')
    else:
        self.failed_count += 1
        self.get_logger().error('Max retries exceeded - dropping')
```

### `publish_heartbeat()`

Sends regular link status.

```python
def publish_heartbeat(self):
    msg = Bool()
    msg.data = True
    self.heartbeat_pub.publish(msg)

    # Publish stats
    stats = String()
    stats.data = f'rx={self.received_count},tx={self.transmitted_count},drop={self.dropped_count}'
    self.stats_pub.publish(stats)
```

## Debugging

### Monitor Transmission

```bash
# Outgoing geotags
ros2 topic echo /gcs/target_report

# Heartbeat
ros2 topic echo /drone1/uplink_heartbeat

# Stats
ros2 topic echo /drone1/uplink_stats
```

### Queue Status

```bash
# Check if backing up
ros2 topic hz /drone1/disease_geotag    # Input rate
ros2 topic hz /gcs/target_report        # Output rate
```

### Common Issues

| Issue            | Cause        | Debug                       |
| ---------------- | ------------ | --------------------------- |
| Queue backing up | Rate too low | Increase `transmit_rate_hz` |
| Drops            | Queue full   | Increase `max_queue_size`   |
| No output        | No input     | Check detection node        |
