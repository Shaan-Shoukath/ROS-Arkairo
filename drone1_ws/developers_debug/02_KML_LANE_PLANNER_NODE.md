# KML Lane Planner Node - Developer Documentation

## Overview

**File**: `drone1_ws/src/kml_lane_planner/kml_lane_planner/kml_lane_planner_node.py`  
**Package**: `kml_lane_planner`  
**Node Name**: `kml_lane_planner_node`  
**Author**: Shaan Shoukath

## Purpose

Parses KML files to generate survey lane waypoints for Drone-1. Converts GPS polygons into parallel flight lanes for systematic field coverage.

## Key Parameters

```yaml
kml_file_path: "field.kml" # Input KML file
lane_spacing_m: 5.0 # Distance between lanes
```

## Subscribers

| Topic                            | Type      | Purpose                 |
| -------------------------------- | --------- | ----------------------- |
| `/mavros/global_position/global` | NavSatFix | Home position reference |

## Publishers

| Topic                    | Type        | Purpose                  |
| ------------------------ | ----------- | ------------------------ |
| `/drone1/lane_segment`   | LaneSegment | Waypoints for navigation |
| `/drone1/mission_status` | String      | Planning status          |

## Lane Generation

```
KML Polygon → Parse coordinates → Generate parallel lanes → Convert to local waypoints → Publish
```

---

**Last Updated**: January 2, 2026
