# Telem RX Node - Developer Documentation

## Overview

**File**: `drone2_ws/src/telem_rx/telem_rx/telem_rx_node.py`  
**Package**: `telem_rx`  
**Node Name**: `telem_rx_node`  
**Author**: Shaan Shoukath

## Purpose

Receives disease geotag telemetry from Drone-1 via MAVLink and republishes as ROS2 NavSatFix messages for the navigation node.

## Key Parameters

```yaml
serial_port: "/dev/ttyUSB0" # Telemetry radio port
baud_rate: 57600 # MAVLink baud rate
```

## Subscribers

None (receives via MAVLink serial)

## Publishers

| Topic                     | Type      | Purpose               |
| ------------------------- | --------- | --------------------- |
| `/drone2/target_position` | NavSatFix | Geotag for navigation |
| `/drone2/telem_status`    | String    | Connection status     |

## Data Flow

```
Drone-1 (telem_tx) → MAVLink Radio → telem_rx → /drone2/target_position → Navigation Node
```

## QoS Configuration

The `/drone2/target_position` publisher uses:
```python
QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)
```

> **Important**: This must match the subscriber in `drone2_navigation_node` to avoid QoS incompatibility warnings.

## Troubleshooting

### QoS Incompatibility with Navigation Node

If you see warnings about incompatible QoS on `/drone2/target_position`:

1. Ensure both publisher (telem_rx) and subscriber (drone2_navigation) use **identical** QoS settings
2. Both must use `BEST_EFFORT` reliability and `TRANSIENT_LOCAL` durability
3. Rebuild both packages after any QoS changes:
   ```bash
   colcon build --packages-select telem_rx drone2_navigation --symlink-install
   ```

---

**Last Updated**: January 5, 2026
