# drone1_msgs - Drone-1 Custom Messages

ROS2 message package for the Survey Drone (Drone-1).

## Messages

### Waypoint.msg

Single GPS waypoint with position data.

### LaneSegment.msg

Survey lane with start/end waypoints and metadata.

### LaneSegmentArray.msg

Array of lane segments for complete mission planning.

## Build

```bash
cd ~/drone1_ws
colcon build --packages-select drone1_msgs
source install/setup.bash
```

## Usage

```python
from drone1_msgs.msg import Waypoint, LaneSegment, LaneSegmentArray
```
