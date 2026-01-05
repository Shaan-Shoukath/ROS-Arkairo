#!/bin/bash

# =============================================================================
# Drone-1 Hardware Launch Script
# =============================================================================
# Opens 3 terminal windows and runs MAVROS, KML Planner, and Navigation
# with appropriate delays between each to allow services to initialize
# =============================================================================

echo "=========================================="
echo "  Drone-1 Hardware Launch"
echo "=========================================="
echo ""
echo "This script will open 3 terminal windows:"
echo "  1. MAVROS"
echo "  2. KML Lane Planner"
echo "  3. Navigation Node"
echo ""

# Configuration
WORKSPACE="/home/pacman/Drone/ROS-Arkairo/drone1_ws"
MISSIONS_FOLDER="${WORKSPACE}/missions"
KML_FILE="SOE.kml"
ALTITUDE=10.0
LANE_SPACING=10.0
LOOKAHEAD_M=12.0
MAX_TARGET_STEP_M=25.0
END_RADIUS_M=5.0

# Timing delays (seconds)
MAVROS_DELAY=5     # Wait for MAVROS to connect to Flight Controller
KML_DELAY=5        # Wait for KML planner to publish

# Detect terminal emulator
if command -v gnome-terminal &> /dev/null; then
    TERMINAL="gnome-terminal"
elif command -v konsole &> /dev/null; then
    TERMINAL="konsole"
elif command -v xterm &> /dev/null; then
    TERMINAL="xterm"
else
    echo "No supported terminal emulator found!"
    echo "Please install gnome-terminal, konsole, or xterm"
    exit 1
fi

echo "Using terminal: $TERMINAL"
echo "Delays: MAVROS=${MAVROS_DELAY}s, KML=${KML_DELAY}s"
echo ""

# ============================================================================
# Terminal 1: MAVROS
# ============================================================================
echo "[1/3] Starting MAVROS..."
if [[ "$TERMINAL" == "gnome-terminal" ]]; then
    gnome-terminal --title="MAVROS" -- bash -c "
        source /opt/ros/jazzy/setup.bash
        echo 'Starting MAVROS...'
        ros2 launch mavros apm.launch fcu_url:=/dev/ttyUSB0:57600
        exec bash
    "
elif [[ "$TERMINAL" == "konsole" ]]; then
    konsole --new-tab -e bash -c "
        source /opt/ros/jazzy/setup.bash
        ros2 launch mavros apm.launch fcu_url:=/dev/ttyUSB0:57600
        exec bash
    " &
else
    xterm -title "MAVROS" -e "bash -c 'source /opt/ros/jazzy/setup.bash && ros2 launch mavros apm.launch fcu_url:=/dev/ttyUSB0:57600; bash'" &
fi

# Wait for MAVROS to connect
echo "   Waiting ${MAVROS_DELAY} seconds for MAVROS to connect..."
sleep ${MAVROS_DELAY}

# ============================================================================
# Terminal 2: KML Lane Planner
# ============================================================================
echo "[2/3] Starting KML Lane Planner..."
if [[ "$TERMINAL" == "gnome-terminal" ]]; then
    gnome-terminal --title="KML Planner" -- bash -c "
        cd ${WORKSPACE}
        source /opt/ros/jazzy/setup.bash
        source install/setup.bash
        echo 'Starting KML Lane Planner...'
        ros2 run kml_lane_planner kml_lane_planner_node --ros-args \
            -p missions_folder:=${MISSIONS_FOLDER} \
            -p kml_filename:=${KML_FILE} \
            -p lane_spacing_m:=${LANE_SPACING} \
            -p altitude_m:=${ALTITUDE} \
            -p enable_republish:=false
        exec bash
    "
elif [[ "$TERMINAL" == "konsole" ]]; then
    konsole --new-tab -e bash -c "
        cd ${WORKSPACE}
        source /opt/ros/jazzy/setup.bash && source install/setup.bash
        ros2 run kml_lane_planner kml_lane_planner_node --ros-args -p missions_folder:=${MISSIONS_FOLDER} -p kml_filename:=${KML_FILE} -p lane_spacing_m:=${LANE_SPACING} -p altitude_m:=${ALTITUDE} -p enable_republish:=false
        exec bash
    " &
else
    xterm -title "KML Planner" -e "bash -c 'cd ${WORKSPACE} && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run kml_lane_planner kml_lane_planner_node --ros-args -p missions_folder:=${MISSIONS_FOLDER} -p kml_filename:=${KML_FILE} -p lane_spacing_m:=${LANE_SPACING} -p altitude_m:=${ALTITUDE} -p enable_republish:=false; bash'" &
fi

# Wait for KML planner to process and publish
echo "   Waiting ${KML_DELAY} seconds for KML planner to publish..."
sleep ${KML_DELAY}

# ============================================================================
# Terminal 3: Navigation Node
# ============================================================================
echo "[3/3] Starting Navigation Node..."
if [[ "$TERMINAL" == "gnome-terminal" ]]; then
    gnome-terminal --title="Navigation" -- bash -c "
        cd ${WORKSPACE}
        source /opt/ros/jazzy/setup.bash
        source install/setup.bash
        echo 'Starting Navigation Node...'
        ros2 run drone1_navigation drone1_navigation_node --ros-args \
            -p takeoff_altitude_m:=${ALTITUDE} \
            -p navigation_altitude_m:=${ALTITUDE} \
            -p path_lookahead_m:=${LOOKAHEAD_M} \
            -p path_max_target_step_m:=${MAX_TARGET_STEP_M} \
            -p path_end_radius_m:=${END_RADIUS_M} \
            -p fcu_timeout_sec:=60.0
        exec bash
    "
elif [[ "$TERMINAL" == "konsole" ]]; then
    konsole --new-tab -e bash -c "
        cd ${WORKSPACE}
        source /opt/ros/jazzy/setup.bash && source install/setup.bash
        ros2 run drone1_navigation drone1_navigation_node --ros-args -p takeoff_altitude_m:=${ALTITUDE} -p navigation_altitude_m:=${ALTITUDE} -p path_lookahead_m:=${LOOKAHEAD_M} -p path_max_target_step_m:=${MAX_TARGET_STEP_M} -p path_end_radius_m:=${END_RADIUS_M} -p fcu_timeout_sec:=60.0
        exec bash
    " &
else
    xterm -title "Navigation" -e "bash -c 'cd ${WORKSPACE} && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run drone1_navigation drone1_navigation_node --ros-args -p takeoff_altitude_m:=${ALTITUDE} -p navigation_altitude_m:=${ALTITUDE} -p path_lookahead_m:=${LOOKAHEAD_M} -p path_max_target_step_m:=${MAX_TARGET_STEP_M} -p path_end_radius_m:=${END_RADIUS_M} -p fcu_timeout_sec:=60.0; bash'" &
fi

echo ""
echo "=========================================="
echo "  All 3 terminals launched!"
echo "=========================================="
echo ""
echo "Total startup time: $((MAVROS_DELAY + KML_DELAY)) seconds"
echo ""
echo "Expected behavior:"
echo "  1. MAVROS connects to Flight Controller via USB"
echo "  2. KML Planner publishes waypoints"
echo "  3. Navigation arms and takes off"
echo ""
echo "Monitor with:"
echo "  ros2 topic echo /drone1/navigation_status"
echo ""
