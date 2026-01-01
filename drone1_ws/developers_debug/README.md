# Drone1 Workspace - Developer Debug Documentation

## Node Documentation Index

This directory contains detailed developer documentation for all Drone-1 ROS2 nodes.

### Navigation

- **[01 - Drone1 Navigation Node](./01_DRONE1_NAVIGATION_NODE.md)**

  - Autonomous flight controller (ARM → TAKEOFF → NAVIGATE → RTL)
  - Package: `drone1_navigation`

- **[02 - KML Lane Planner Node](./02_KML_LANE_PLANNER_NODE.md)**

  - Mission planning from KML polygon files
  - Package: `kml_lane_planner`

- **[03 - Image Capture Node](./03_IMAGE_CAPTURE_NODE.md)**

  - USB camera interface and image publishing
  - Package: `image_capture`

- **[04 - Detection and Geotag Node](./04_DETECTION_AND_GEOTAG_NODE.md)**

  - Yellow disease detection + GPS geotagging
  - Package: `detection_and_geotag`

- **[05 - Telemetry TX Node](./05_TELEM_TX_NODE.md)**
  - MAVLink telemetry transmission to Drone-2
  - Package: `telem_tx`

## System Architecture

```
KML File → Lane Planner → Navigation → Drone Flight
                              ↓
Camera → Image Capture → Detection → Geotag → Telem TX → [Radio] → Drone-2
```

## Quick Reference

| Node                     | Purpose           | Key Topics                                                  |
| ------------------------ | ----------------- | ----------------------------------------------------------- |
| **drone1_navigation**    | Flight control    | `/mission/lane_segments`, `/mavros/setpoint_position/local` |
| **kml_lane_planner**     | Mission planning  | `/mission/lane_segments`                                    |
| **image_capture**        | Camera feed       | `/camera/image_raw`, `/camera/camera_info`                  |
| **detection_and_geotag** | Disease detection | `/camera/image_raw` → `/drone1/disease_geotag`              |
| **telem_tx**             | Radio TX          | `/drone1/disease_geotag` → `/mavros/debug_value/send`       |

## Documentation Structure

Each node document includes:

- ✅ **Overview**: File path, package, purpose
- ✅ **What It Does**: High-level functionality
- ✅ **Core Logic & Reasoning**: Design decisions explained
- ✅ **Subscribers**: All input topics with sources
- ✅ **Publishers**: All output topics with targets
- ✅ **Parameters**: Configuration table
- ✅ **Key Functions**: Detailed explanations with built-in function docs
- ✅ **Package Dependencies**: ROS2 + Python libraries
- ✅ **Error Handling**: Common issues and solutions
- ✅ **Testing Checklist**: Validation steps

## Getting Started

1. Read the navigation node documentation first to understand the flight controller
2. Follow the data flow: Planner → Navigation → Detection → Telemetry
3. Each document is self-contained with all dependencies explained

## For New Developers

**Start Here**:

1. [Drone1 Navigation Node](./01_DRONE1_NAVIGATION_NODE.md) - Core flight control
2. [Detection and Geotag Node](./04_DETECTION_AND_GEOTAG_NODE.md) - Computer vision explained

**Then Explore**:

- Mission planning algorithms
- Camera interfacing
- MAVLink telemetry protocol

---

**Last Updated**: December 31, 2025  
**Maintained by**: Shaan Shoukath
