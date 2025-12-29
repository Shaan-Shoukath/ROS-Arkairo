# KML Lane Planner Node

**Automatic Mission Generation from KML Boundaries**

## Overview

Monitors `missions/` folder for KML polygon files. When detected, automatically generates lawnmower survey waypoints and triggers the full autonomous mission sequence.

## Pipeline

```
KML File Detected
      │
      ▼
Parse Polygon Coordinates (WGS84)
      │
      ▼
Convert to ENU (local metric)
      │
      ▼
Find Longest Edge → Determine Sweep Direction
      │
      ▼
Generate Parallel Lines (lawnmower pattern)
      │
      ▼
Intersect with Polygon Boundary
      │
      ▼
Optimize for Home Position (reverse if needed)
      │
      ▼
Publish LaneSegmentArray → Triggers Navigation
```

## Topics

### Subscribers

| Topic                            | Type        | Description                    |
| -------------------------------- | ----------- | ------------------------------ |
| `/mavros/global_position/global` | `NavSatFix` | Home position for optimization |
| `/mission/load_kml`              | `String`    | Manual KML path trigger        |

### Publishers

| Topic                    | Type               | Description                 |
| ------------------------ | ------------------ | --------------------------- |
| `/mission/lane_segments` | `LaneSegmentArray` | Survey lanes for navigation |
| `/drone1/next_waypoint`  | `NavSatFix`        | Current target waypoint     |

## Parameters

| Parameter           | Default        | Description                    |
| ------------------- | -------------- | ------------------------------ |
| `missions_folder`   | `.../missions` | KML watch directory            |
| `altitude_m`        | `6.7`          | Survey altitude (22 feet)      |
| `lane_spacing_m`    | `5.0`          | Distance between lanes         |
| `buffer_distance_m` | `2.0`          | Inward buffer from boundary    |
| `require_gps_home`  | `false`        | Wait for GPS before processing |
| `default_home_lat`  | `10.0478`      | Fallback home latitude         |
| `default_home_lon`  | `76.3303`      | Fallback home longitude        |

## Coordinate Transforms

### WGS84 to ENU

```python
# Earth-Centered Earth-Fixed (ECEF) intermediate
X = (N + h)·cos(lat)·cos(lon)
Y = (N + h)·cos(lat)·sin(lon)
Z = (N·(1-e²) + h)·sin(lat)

# Rotation to East-North-Up
E = -sin(lon₀)·ΔX + cos(lon₀)·ΔY
N = -sin(lat₀)·cos(lon₀)·ΔX - sin(lat₀)·sin(lon₀)·ΔY + cos(lat₀)·ΔZ
U = cos(lat₀)·cos(lon₀)·ΔX + cos(lat₀)·sin(lon₀)·ΔY + sin(lat₀)·ΔZ
```

### Line-Polygon Intersection

```python
# Parametric line intersection for sweep line generation
t = ((x1-x3)(y3-y4) - (y1-y3)(x3-x4)) / denom
u = -((x1-x2)(y1-y3) - (y1-y2)(x1-x3)) / denom
if 0 ≤ u ≤ 1: intersection = (x1 + t(x2-x1), y1 + t(y2-y1))
```

## KML Format

```xml
<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <Placemark>
      <Polygon>
        <outerBoundaryIs>
          <LinearRing>
            <coordinates>
              lon1,lat1,0
              lon2,lat2,0
              ...
            </coordinates>
          </LinearRing>
        </outerBoundaryIs>
      </Polygon>
    </Placemark>
  </Document>
</kml>
```

## Usage

```bash
# Launch as part of survey system
ros2 launch drone1_bringup drone1_survey.launch.py

# Drop KML to trigger mission
cp field.kml ~/Documents/ROSArkairo/drone1_ws/missions/
```

## Home Position Optimization

The algorithm optimizes the survey starting point:

1. Computes distance from home to first waypoint
2. Computes distance from home to last waypoint
3. If last is 80%+ closer, reverses the waypoint order
4. Minimizes travel distance at mission start
