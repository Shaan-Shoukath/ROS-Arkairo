# Drone2 Workspace - Developer Debug Documentation

## Node Documentation Index

This directory contains detailed developer documentation for all Drone-2 ROS2 nodes.

### Navigation

- **[01 - Telemetry RX Node](./01_TELEM_RX_NODE.md)**

  - MAVLink telemetry receiver from Drone-1
  - Package: `telem_rx`

- **[02 - Drone2 Navigation Node](./02_DRONE2_NAVIGATION_NODE.md)** ⚡ **UNIFIED**

  - Complete autonomous flight controller (replaces mission_manager)
  - Package: `drone2_navigation`

- **[03 - Detection & Centering Node](./03_DETECTION_CENTERING_NODE.md)** ⚡ **MERGED**

  - Combined detection + visual servoing (single node)
  - Package: `local_detection_and_centering`

- **[04 - Sprayer Control Node](./04_SPRAYER_CONTROL_NODE.md)**
  - Spray pump control with safety checks
  - Package: `sprayer_control`

## System Architecture (NEW - Unified)

```
Drone-1 Radio → Telem RX → Navigation → ARM/TAKEOFF/FLY
                                ↓
                            Arrival Status
                                ↓
                    Detection & Centering (merged)
                                ↓
                           Spray Ready
                                ↓
                        Sprayer Control
                                ↓
                          Spray Complete
                                ↓
                    Navigation (wait/next target)
```

## 🚨 Important Changes

### Unified Navigation (Dec 2025)

The `mission_manager` node has been **merged into** `drone2_navigation_node`.

**Old (Broken)**:

```
telem_rx → drone2_navigation (no ARM/TAKEOFF)
        → mission_manager (ARM/TAKEOFF but never triggered)
```

**New (Working)**:

```
telem_rx → drone2_navigation (complete: ARM → TAKEOFF → NAV → SPRAY cycle)
```

**Action Required**:

- ✅ Do NOT launch `mission_manager_node`
- ✅ Launch only `drone2_navigation_node`
- ✅ See updated `drone2_sprayer.launch.py`

### Merged Detection & Centering

The separate `local_detection` and `centering_controller` nodes have been **merged** into `detection_centering_node`.

**Benefits**:

- Faster response (no topic latency)
- Simpler state management
- Reduced system complexity

## Quick Reference

| Node                       | Purpose                | Key Topics                                                    |
| -------------------------- | ---------------------- | ------------------------------------------------------------- |
| **telem_rx**               | Receive targets        | `/mavros/debug_value/recv` → `/drone2/target_position`        |
| **drone2_navigation** ⚡   | Unified flight control | `/drone2/target_position` → `/mavros/setpoint_position/local` |
| **detection_centering** ⚡ | Find & center disease  | `/drone2/arrival_status` → `/drone2/spray_ready`              |
| **sprayer_control**        | Pump actuation         | `/drone2/spray_ready` → `/drone2/spray_done`                  |

⚡ = Recently unified/merged nodes

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

## Data Flow

```
1. Drone-1 detects disease → Transmits GPS
2. Telem RX receives → Validates → Publishes target
3. Navigation receives target → ARMS → TAKEOFF → FLY to target
4. Arrival at target → Triggers detection/centering
5. Detection finds disease → Centers drone
6. Spray ready → Activates pump
7. Spray complete → Navigation waits for next target (or RTL)
```

## Getting Started

**Essential Reading (in order)**:

1. [Telem RX Node](./01_TELEM_RX_NODE.md) - How targets are received
2. [Drone2 Navigation Node](./02_DRONE2_NAVIGATION_NODE.md) - Unified flight controller ⚡
3. [Detection & Centering Node](./03_DETECTION_CENTERING_NODE.md) - Visual servoing ⚡
4. [Sprayer Control Node](./04_SPRAYER_CONTROL_NODE.md) - Pump control

## For New Developers

**Critical Concepts**:

- **Unified Navigation**: Single node handles entire flight (not split anymore)
- **Merged Detection**: One node does find + center (not two separate)
- **State Machines**: Each node uses FSM (Finite State Machine)
- **Safety Checks**: Multiple layers before spray activation

**Start Here**:

1. Read Drone2 Navigation (the brain)
2. Understand Detection & Centering (the eyes + control)
3. Review Sprayer Control (the actuator)

---

**Last Updated**: December 31, 2025  
**Maintained by**: Shaan Shoukath  
**⚠️ Architecture updated**: Mission manager merged, detection merged
