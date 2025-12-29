#!/bin/bash

# =============================================================================
# Drone-1 Full System Launch Script
# =============================================================================
# Opens 4 terminal windows and runs SITL, MAVROS, KML Planner, and Navigation
# with appropriate delays between each to allow services to initialize
# =============================================================================

echo "=========================================="
echo "  Drone-1 Full System Launch"
echo "=========================================="
echo ""
echo "This script will open 4 terminal windows:"
echo "  1. ArduPilot SITL (Kerala location)"
echo "  2. MAVROS"
echo "  3. KML Lane Planner"
echo "  4. Navigation Node"
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
VENV_PATH="$HOME/venv-ardupilot/bin/activate"

# Timing delays (seconds)
SITL_DELAY=30      # Wait for SITL to fully initialize + GPS lock
MAVROS_DELAY=15    # Wait for MAVROS to connect to SITL
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
echo "Delays: SITL=${SITL_DELAY}s, MAVROS=${MAVROS_DELAY}s, KML=${KML_DELAY}s"
echo ""

# ============================================================================
# Terminal 1: SITL (with venv activation)
# ============================================================================
echo "[1/4] Starting ArduPilot SITL..."
if [[ "$TERMINAL" == "gnome-terminal" ]]; then
    gnome-terminal --title="SITL - ArduCopter" -- bash -c "
        source ${VENV_PATH}
        echo 'ArduPilot venv activated'
        echo 'Starting ArduPilot SITL at Kerala location...'
        sim_vehicle.py -v ArduCopter -f quad \
            --console \
            --map \
            --out=udp:127.0.0.1:14551 \
            --no-rebuild \
            --custom-location=10.0483,76.331,0,0
        exec bash
    "
elif [[ "$TERMINAL" == "konsole" ]]; then
    konsole --new-tab -e bash -c "
        source ${VENV_PATH}
        sim_vehicle.py -v ArduCopter -f quad --console --map --out=udp:127.0.0.1:14551 --no-rebuild --custom-location=10.0483,76.331,0,0
        exec bash
    " &
else
    xterm -title "SITL" -e "bash -c 'source ${VENV_PATH} && sim_vehicle.py -v ArduCopter -f quad --console --map --out=udp:127.0.0.1:14551 --no-rebuild --custom-location=10.0483,76.331,0,0; bash'" &
fi

# Wait for SITL to initialize (increased to 30s for GPS lock)
echo "   Waiting ${SITL_DELAY} seconds for SITL to start and GPS to lock..."
sleep ${SITL_DELAY}

# ============================================================================
# Terminal 2: MAVROS
# ============================================================================
echo "[2/4] Starting MAVROS..."
if [[ "$TERMINAL" == "gnome-terminal" ]]; then
    gnome-terminal --title="MAVROS" -- bash -c "
        source /opt/ros/jazzy/setup.bash
        echo 'Starting MAVROS...'
        ros2 launch mavros apm.launch fcu_url:=udp://:14551@127.0.0.1:14550
        exec bash
    "
elif [[ "$TERMINAL" == "konsole" ]]; then
    konsole --new-tab -e bash -c "
        source /opt/ros/jazzy/setup.bash
        ros2 launch mavros apm.launch fcu_url:=udp://:14551@127.0.0.1:14550
        exec bash
    " &
else
    xterm -title "MAVROS" -e "bash -c 'source /opt/ros/jazzy/setup.bash && ros2 launch mavros apm.launch fcu_url:=udp://:14551@127.0.0.1:14550; bash'" &
fi

# Wait for MAVROS to connect (increased to 15s)
echo "   Waiting ${MAVROS_DELAY} seconds for MAVROS to connect..."
sleep ${MAVROS_DELAY}

# ============================================================================
# Terminal 3: KML Lane Planner
# ============================================================================
echo "[3/4] Starting KML Lane Planner..."
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
# Terminal 4: Navigation Node
# ============================================================================
echo "[4/4] Starting Navigation Node..."
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
echo "  All 4 terminals launched!"
echo "=========================================="
echo ""
echo "Total startup time: $((SITL_DELAY + MAVROS_DELAY + KML_DELAY)) seconds"
echo ""
echo "Expected behavior:"
echo "  1. SITL starts at Kerala (10.0483, 76.331)"
echo "  2. MAVROS connects to SITL"
echo "  3. KML Planner publishes waypoints"
echo "  4. Navigation arms and takes off"
echo ""
echo "Monitor with:"
echo "  ros2 topic echo /drone1/navigation_status"
echo ""
