# KML Lane Planner Node - Developer Documentation

## Overview

| Property        | Value                                                                      |
| --------------- | -------------------------------------------------------------------------- |
| **File**        | `drone1_ws/src/kml_lane_planner/kml_lane_planner/kml_lane_planner_node.py` |
| **Package**     | `kml_lane_planner`                                                         |
| **Node Name**   | `kml_lane_planner_node`                                                    |
| **Config File** | `kml_lane_planner/config/planner_params.yaml`                              |
| **Maintainer**  | Shaan Shoukath                                                             |

---

## Purpose

Parses KML files to generate survey lane waypoints for Drone 1. Converts GPS polygons into parallel flight lanes (lawnmower pattern) for systematic field coverage with configurable lane spacing and boundary buffer.

---

## State Variables

| Variable           | Type                 | Purpose                                 |
| ------------------ | -------------------- | --------------------------------------- |
| `missions_folder`  | `str`                | Path to folder containing KML files     |
| `altitude`         | `float`              | Flight altitude in meters (6.7m = 22ft) |
| `spacing`          | `float`              | Distance between parallel lanes (10m)   |
| `buffer_distance`  | `float`              | Safety padding from polygon edges (2m)  |
| `home_gps`         | `Tuple[float,float]` | Home latitude/longitude                 |
| `polygon`          | `List[Tuple]`        | Current KML polygon coordinates         |
| `waypoints`        | `List[Tuple]`        | Generated waypoints (lat, lon, alt)     |
| `current_kml_file` | `str`                | Currently loaded KML filename           |
| `require_gps_home` | `bool`               | Wait for GPS or use config home         |

---

## Configuration YAML → Code Mapping

```yaml
# planner_params.yaml                      # Code Usage
missions_folder: "~/Documents/.../missions" →  os.path.expanduser(self.missions_folder)
altitude_m: 6.7                          →  self.altitude (flight height)
lane_spacing_m: 10.0                     →  self.spacing (between parallel lines)
buffer_distance_m: 2.0                   →  self.buffer_distance (from edges)
watch_interval_sec: 2.0                  →  Timer period for file watching
require_gps_home: false                  →  self.require_gps_home
default_home_lat: 28.4215                →  self.default_home_lat
default_home_lon: 77.5243                →  self.default_home_lon
```

---

## Key Functions and Why They Exist

### Core Functions

| Function              | Why It Exists                                    | Variables Used         |
| --------------------- | ------------------------------------------------ | ---------------------- |
| `__init__`            | Initialize ROS, declare params, create pubs/subs | All config params      |
| `watch_for_kml_files` | Timer callback to scan for new KML files         | `missions_folder`      |
| `process_kml_file`    | Parse KML and generate waypoints                 | `polygon`, `waypoints` |
| `generate_waypoints`  | Create lawnmower pattern from polygon            | `spacing`, `altitude`  |
| `publish_mission`     | Send waypoints to navigation node                | `waypoints`            |

### Geometry Functions

| Function                             | Why It Exists                            | Variables Used    |
| ------------------------------------ | ---------------------------------------- | ----------------- |
| `create_buffer_polygon`              | Shrink polygon inward by buffer_distance | `buffer_distance` |
| `find_polygon_corners`               | Identify NE, NW, SE, SW corners          | `polygon`         |
| `line_polygon_intersections`         | Calculate where scan lines cross polygon | -                 |
| `optimize_line_ordering_for_corners` | Order waypoints for efficient path       | `waypoints`       |

### Coordinate Functions

| Function                   | Why It Exists                      | Variables Used |
| -------------------------- | ---------------------------------- | -------------- |
| `latlon_to_enu`            | Convert GPS to local East-North-Up | `home_gps`     |
| `enu_to_latlon`            | Convert ENU back to GPS            | `home_gps`     |
| `_line_intersection_point` | Find where two lines intersect     | -              |

---

## Subscribers

| Topic                            | Type        | Callback       | Purpose               |
| -------------------------------- | ----------- | -------------- | --------------------- |
| `/mavros/global_position/global` | `NavSatFix` | `gps_callback` | Get home GPS position |

## Publishers

| Topic                    | Type               | Purpose                  |
| ------------------------ | ------------------ | ------------------------ |
| `/drone1/lane_segment`   | `LaneSegmentArray` | Waypoints for navigation |
| `/drone1/mission_status` | `String`           | Planning status messages |

---

## Lawnmower Pattern Generation

```
Original KML Polygon:
┌─────────────────────────────────────┐
│                                     │
│                                     │
│                                     │
│                                     │
│                                     │
└─────────────────────────────────────┘

After Buffer (2m inward):
  ┌─────────────────────────────────┐
  │                                 │
  │                                 │
  │                                 │
  │                                 │
  └─────────────────────────────────┘

Generated Lanes (10m spacing):
  ┌─────────────────────────────────┐
  │ ─────────────────────────────── │ Lane 1
  │                                 │
  │ ─────────────────────────────── │ Lane 2 (reversed)
  │                                 │
  │ ─────────────────────────────── │ Lane 3
  └─────────────────────────────────┘

Connected Path (snake pattern):
  ┌─────────────────────────────────┐
  │ ────────────────────────────┐   │
  │                             │   │
  │ ┌───────────────────────────┘   │
  │ │                               │
  │ └──────────────────────────►    │
  └─────────────────────────────────┘
```

---

## Buffer Polygon Algorithm

Edge-based inward offset ensures ALL sides shrink uniformly:

```python
# For each edge:
1. Calculate edge direction vector
2. Compute perpendicular inward normal
3. Offset edge by buffer_distance along normal
4. Find new vertices at edge intersections

# Result: Polygon shrunk uniformly on all sides
```

---

## Data Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    KML LANE PLANNER                         │
└─────────────────────────────────────────────────────────────┘

~/missions/SOE.kml
     │
     │ (file watcher detects new/changed KML)
     ▼
┌─────────────────────────┐
│   kml_lane_planner_node │
│                         │
│  1. Parse KML polygon   │
│  2. Apply buffer        │
│  3. Generate lanes      │
│  4. Optimize ordering   │
│  5. Publish waypoints   │
└─────────────┬───────────┘
              │ LaneSegmentArray
              │ /drone1/lane_segment
              ▼
     drone1_navigation_node
              │
              │ Converts to local ENU
              ▼
         Flies pattern
```

---

## Manual Testing

### Run KML Planner

```bash
cd ~/Documents/ROS-Arkairo/drone1_ws && source install/setup.zsh
ros2 run kml_lane_planner kml_lane_planner_node --ros-args \
  --params-file ~/Documents/ROS-Arkairo/drone1_ws/src/kml_lane_planner/config/planner_params.yaml
```

### Place a KML File

```bash
cp your_field.kml ~/Documents/ROS-Arkairo/drone1_ws/missions/
```

### Watch Generated Waypoints

```bash
ros2 topic echo /drone1/lane_segment
```

### Monitor Status

```bash
ros2 topic echo /drone1/mission_status
```

---

## Debugging

### Common Issues

| Issue                      | Cause                            | Solution                     |
| -------------------------- | -------------------------------- | ---------------------------- |
| "No KML files found"       | Wrong path or tilde not expanded | Check `missions_folder` path |
| 0 waypoints generated      | Polygon too small or invalid     | Check KML coordinates        |
| Waypoints outside field    | Buffer too large                 | Reduce `buffer_distance_m`   |
| Pattern not covering field | Lane spacing too wide            | Reduce `lane_spacing_m`      |

### Debug Logging

```bash
# Enable debug output
ros2 run kml_lane_planner kml_lane_planner_node --ros-args \
  --log-level debug \
  --params-file ...
```

---

## Config File Location

```
drone1_ws/src/kml_lane_planner/config/planner_params.yaml
```

```yaml
/**:
  ros__parameters:
    missions_folder: "~/Documents/ROS-Arkairo/drone1_ws/missions"
    altitude_m: 6.7
    lane_spacing_m: 10.0
    buffer_distance_m: 2.0
    watch_interval_sec: 2.0
    require_gps_home: false
    default_home_lat: 28.4215
    default_home_lon: 77.5243
```

---

**Last Updated**: January 13, 2026
