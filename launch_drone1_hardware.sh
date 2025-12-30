#!/bin/bash

# =============================================================================
# Drone-1 Hardware Launch Script
# =============================================================================
# Launches MAVROS and ROS2 nodes for Cube Orange+ hardware deployment
# Hardware: Cube Orange+ FCU | Companion: Raspberry Pi 5 | ROS: Jazzy
# =============================================================================

set -e

echo "=========================================="
echo "  Drone-1 Hardware Launch"
echo "  Cube Orange+ | Raspberry Pi 5"
echo "=========================================="
echo ""

# =============================================================================
# CONFIGURATION - Adjust for your hardware setup
# =============================================================================

# Serial connection to Cube Orange+ (TELEM2 port)
# Options: /dev/ttyAMA0 (GPIO UART) or /dev/ttyACM0 (USB)
FCU_SERIAL_PORT="${FCU_SERIAL_PORT:-/dev/ttyAMA0}"
FCU_BAUD_RATE="${FCU_BAUD_RATE:-921600}"

# Optional GCS passthrough (set to empty to disable)
GCS_URL="${GCS_URL:-}"

# Workspace path
WORKSPACE="${WORKSPACE:-/home/$(whoami)/ROS-Arkairo/drone1_ws}"

# Mission parameters
MISSIONS_FOLDER="${WORKSPACE}/missions"
KML_FILE="${KML_FILE:-}"  # Leave empty to auto-detect

# Flight parameters (safe defaults for real flight)
ALTITUDE=6.7              # 22 feet - safe initial test altitude
LANE_SPACING=10.0
LOOKAHEAD_M=12.0
MAX_TARGET_STEP_M=25.0
END_RADIUS_M=7.0          # Larger for GPS error

# =============================================================================
# PRE-FLIGHT CHECKS
# =============================================================================

echo "[PRE-FLIGHT] Running hardware checks..."
echo ""

# Check 1: Serial port exists
if [[ ! -e "$FCU_SERIAL_PORT" ]]; then
    echo "❌ ERROR: Serial port $FCU_SERIAL_PORT not found!"
    echo "   Available ports:"
    ls /dev/ttyUSB* /dev/ttyACM* /dev/ttyAMA* 2>/dev/null || echo "   (none found)"
    echo ""
    echo "   Check Cube Orange+ connection and try again."
    exit 1
fi
echo "✅ Serial port: $FCU_SERIAL_PORT"

# Check 2: Workspace exists
if [[ ! -d "$WORKSPACE" ]]; then
    echo "❌ ERROR: Workspace not found: $WORKSPACE"
    exit 1
fi
echo "✅ Workspace: $WORKSPACE"

# Check 3: ROS2 setup files exist
if [[ ! -f "/opt/ros/jazzy/setup.bash" ]]; then
    echo "❌ ERROR: ROS2 Jazzy not found at /opt/ros/jazzy"
    exit 1
fi
echo "✅ ROS2 Jazzy installation found"

# Check 4: Workspace built
if [[ ! -f "$WORKSPACE/install/setup.bash" ]]; then
    echo "❌ ERROR: Workspace not built. Run:"
    echo "   cd $WORKSPACE && colcon build --symlink-install"
    exit 1
fi
echo "✅ Workspace built"

# Check 5: Serial port permissions
if [[ ! -r "$FCU_SERIAL_PORT" ]] || [[ ! -w "$FCU_SERIAL_PORT" ]]; then
    echo "⚠️  WARNING: Serial port permissions may be insufficient"
    echo "   Run: sudo chmod 666 $FCU_SERIAL_PORT"
    echo "   Or add user to dialout group: sudo usermod -a -G dialout \$USER"
fi

echo ""
echo "=========================================="
echo "  Pre-flight checks PASSED"
echo "=========================================="
echo ""

# =============================================================================
# ENVIRONMENT SETUP
# =============================================================================

# Source ROS2
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/install/setup.bash"

# Set hardware mode environment variable
export ROS_ARKAIRO_HARDWARE=true

# =============================================================================
# BUILD FCU URL
# =============================================================================

FCU_URL="${FCU_SERIAL_PORT}:${FCU_BAUD_RATE}"
echo "FCU URL: $FCU_URL"

if [[ -n "$GCS_URL" ]]; then
    echo "GCS Passthrough: $GCS_URL"
fi
echo ""

# =============================================================================
# LAUNCH SEQUENCE
# =============================================================================

echo "[1/2] Starting MAVROS..."
echo "      Connected to Cube Orange+ via $FCU_SERIAL_PORT"
echo ""

# Build MAVROS launch command
MAVROS_CMD="ros2 launch mavros apm.launch fcu_url:=$FCU_URL"
if [[ -n "$GCS_URL" ]]; then
    MAVROS_CMD="$MAVROS_CMD gcs_url:=$GCS_URL"
fi

# Start MAVROS in background
$MAVROS_CMD &
MAVROS_PID=$!

# Wait for MAVROS to connect
echo "   Waiting for FCU connection..."
sleep 5

# Check if MAVROS is still running
if ! kill -0 $MAVROS_PID 2>/dev/null; then
    echo "❌ ERROR: MAVROS failed to start"
    echo "   Check serial connection and baud rate"
    exit 1
fi

echo "✅ MAVROS started (PID: $MAVROS_PID)"
echo ""

# =============================================================================
# LAUNCH ROS2 NODES
# =============================================================================

echo "[2/2] Starting Drone-1 Survey System (Hardware Mode)..."
echo ""

# Build navigation parameters
NAV_ARGS="--ros-args"
NAV_ARGS="$NAV_ARGS -p takeoff_altitude_m:=${ALTITUDE}"
NAV_ARGS="$NAV_ARGS -p navigation_altitude_m:=${ALTITUDE}"
NAV_ARGS="$NAV_ARGS -p path_lookahead_m:=${LOOKAHEAD_M}"
NAV_ARGS="$NAV_ARGS -p path_max_target_step_m:=${MAX_TARGET_STEP_M}"
NAV_ARGS="$NAV_ARGS -p path_end_radius_m:=${END_RADIUS_M}"
NAV_ARGS="$NAV_ARGS -p waypoint_arrival_radius_m:=5.0"
NAV_ARGS="$NAV_ARGS -p fcu_timeout_sec:=60.0"
NAV_ARGS="$NAV_ARGS -p hardware_mode:=true"

# Launch with hardware flag
ros2 launch drone1_bringup drone1_survey.launch.py hardware:=true &
NODES_PID=$!

echo "✅ Drone-1 Survey System started (PID: $NODES_PID)"
echo ""

# =============================================================================
# STATUS
# =============================================================================

echo "=========================================="
echo "  Hardware Launch Complete"
echo "=========================================="
echo ""
echo "Monitor commands:"
echo "  ros2 topic echo /mavros/state"
echo "  ros2 topic echo /drone1/navigation_status"
echo "  ros2 topic echo /mavros/global_position/global"
echo ""
echo "Emergency commands:"
echo "  ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \"{custom_mode: 'RTL'}\""
echo "  ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \"{value: false}\""
echo ""
echo "Press Ctrl+C to stop all nodes..."
echo ""

# =============================================================================
# CLEANUP HANDLER
# =============================================================================

cleanup() {
    echo ""
    echo "Shutting down..."
    
    # Kill ROS nodes
    if [[ -n "$NODES_PID" ]] && kill -0 $NODES_PID 2>/dev/null; then
        kill $NODES_PID 2>/dev/null
    fi
    
    # Kill MAVROS
    if [[ -n "$MAVROS_PID" ]] && kill -0 $MAVROS_PID 2>/dev/null; then
        kill $MAVROS_PID 2>/dev/null
    fi
    
    echo "All processes stopped."
    exit 0
}

trap cleanup SIGINT SIGTERM

# Wait for processes
wait
