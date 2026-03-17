#!/bin/bash

# =============================================================================
# Drone-1 Full System Launch Script (with Detection)
# =============================================================================
# Opens 6 terminal windows and runs the complete Drone-1 system:
#   1. ArduPilot SITL
#   2. MAVROS
#   3. KML Lane Planner
#   4. Navigation Node
#   5. Image Capture + Detection + Telem TX
#   6. Optional: Monitor terminal
# =============================================================================

echo "==========================================="
echo "  Drone-1 Full System Launch (Detection)"
echo "==========================================="
echo ""
echo "This script will open 5 terminal windows:"
echo "  1. ArduPilot SITL (Kerala location)"
echo "  2. MAVROS"
echo "  3. KML Lane Planner"
echo "  4. Navigation Node"
echo "  5. Image Capture + Detection + Telem TX"
echo ""

# Configuration
WORKSPACE="/home/shaan-shoukath/Documents/ROS-Arkairo/drone1_ws"
MISSIONS_FOLDER="${WORKSPACE}/missions"
KML_FILE="SOE.kml"
ALTITUDE=10.0
LANE_SPACING=10.0
LOOKAHEAD_M=12.0
MAX_TARGET_STEP_M=25.0
END_RADIUS_M=5.0
USE_SIM=true  # Set to true for laptop webcam, false for Pi Camera

# Kerala coordinates
HOME_LAT=10.0483
HOME_LON=76.331

# Timing delays (seconds)
SITL_DELAY=30      # Wait for SITL to fully initialize + GPS lock
MAVROS_DELAY=15    # Wait for MAVROS to connect to SITL
KML_DELAY=5        # Wait for KML planner to publish
NAV_DELAY=3        # Wait before starting detection nodes

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
echo "USE_SIM: ${USE_SIM} (laptop webcam)"
echo ""

# ============================================================================
# Terminal 1: SITL
# ============================================================================
echo "[1/5] Starting ArduPilot SITL..."
if [[ "$TERMINAL" == "gnome-terminal" ]]; then
    gnome-terminal --title="SITL - ArduCopter" -- bash -c "
        echo 'Starting ArduPilot SITL at Kerala location...'
        cd ~/ardupilot/ArduCopter
        sim_vehicle.py -v ArduCopter -f quad \\
            --console \\
            --map \\
            --out=udp:127.0.0.1:14551 \\
            --no-rebuild \\
            --custom-location=${HOME_LAT},${HOME_LON},0,0
        exec bash
    "
elif [[ "$TERMINAL" == "konsole" ]]; then
    konsole --new-tab -e bash -c "
        cd ~/ardupilot/ArduCopter
        sim_vehicle.py -v ArduCopter -f quad --console --map --out=udp:127.0.0.1:14551 --no-rebuild --custom-location=${HOME_LAT},${HOME_LON},0,0
        exec bash
    " &
else
    xterm -title "SITL" -e "bash -c 'cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f quad --console --map --out=udp:127.0.0.1:14551 --no-rebuild --custom-location=${HOME_LAT},${HOME_LON},0,0; bash'" &
fi

echo "   Waiting ${SITL_DELAY} seconds for SITL to start and GPS to lock..."
sleep ${SITL_DELAY}

# ============================================================================
# Terminal 2: MAVROS
# ============================================================================
echo "[2/5] Starting MAVROS..."
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

echo "   Waiting ${MAVROS_DELAY} seconds for MAVROS to connect..."
sleep ${MAVROS_DELAY}

# ============================================================================
# Terminal 3: KML Lane Planner
# ============================================================================
echo "[3/5] Starting KML Lane Planner..."
if [[ "$TERMINAL" == "gnome-terminal" ]]; then
    gnome-terminal --title="KML Planner" -- bash -c "
        cd ${WORKSPACE}
        source /opt/ros/jazzy/setup.bash
        source install/setup.bash
        echo 'Starting KML Lane Planner...'
        ros2 run kml_lane_planner kml_lane_planner_node --ros-args \\
            -p missions_folder:=${MISSIONS_FOLDER} \\
            -p kml_filename:=${KML_FILE} \\
            -p lane_spacing_m:=${LANE_SPACING} \\
            -p altitude_m:=${ALTITUDE} \\
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

echo "   Waiting ${KML_DELAY} seconds for KML planner to publish..."
sleep ${KML_DELAY}

# ============================================================================
# Terminal 4: Navigation Node
# ============================================================================
echo "[4/5] Starting Navigation Node..."
if [[ "$TERMINAL" == "gnome-terminal" ]]; then
    gnome-terminal --title="Navigation" -- bash -c "
        cd ${WORKSPACE}
        source /opt/ros/jazzy/setup.bash
        source install/setup.bash
        echo 'Starting Navigation Node...'
        ros2 run drone1_navigation drone1_navigation_node --ros-args \\
            -p takeoff_altitude_m:=${ALTITUDE} \\
            -p navigation_altitude_m:=${ALTITUDE} \\
            -p path_lookahead_m:=${LOOKAHEAD_M} \\
            -p path_max_target_step_m:=${MAX_TARGET_STEP_M} \\
            -p path_end_radius_m:=${END_RADIUS_M} \\
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

echo "   Waiting ${NAV_DELAY} seconds before starting detection nodes..."
sleep ${NAV_DELAY}

# ============================================================================
# Terminal 5: Image Capture + Detection + Telem TX
# ============================================================================
echo "[5/5] Starting Detection Pipeline (Image Capture + Detection + Telem TX)..."
if [[ "$TERMINAL" == "gnome-terminal" ]]; then
    gnome-terminal --title="Detection Pipeline" -- bash -c "
        cd ${WORKSPACE}
        source /opt/ros/jazzy/setup.bash
        source install/setup.bash
        echo '=========================================='
        echo ' Detection Pipeline'
        echo '=========================================='
        echo 'Starting: Image Capture, Detection, Telem TX'
        echo 'USE_SIM: ${USE_SIM}'
        echo ''
        
        # Start all three nodes
        ros2 run image_capture image_capture_node --ros-args -p use_sim:=${USE_SIM} &
        sleep 2
        ros2 run detection_and_geotag detection_and_geotag_node &
        sleep 1
        ros2 run telem_tx telem_tx_node
        
        exec bash
    "
elif [[ "$TERMINAL" == "konsole" ]]; then
    konsole --new-tab -e bash -c "
        cd ${WORKSPACE}
        source /opt/ros/jazzy/setup.bash && source install/setup.bash
        ros2 run image_capture image_capture_node --ros-args -p use_sim:=${USE_SIM} &
        sleep 2
        ros2 run detection_and_geotag detection_and_geotag_node &
        sleep 1
        ros2 run telem_tx telem_tx_node
        exec bash
    " &
else
    xterm -title "Detection" -e "bash -c 'cd ${WORKSPACE} && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run image_capture image_capture_node --ros-args -p use_sim:=${USE_SIM} & sleep 2 && ros2 run detection_and_geotag detection_and_geotag_node & sleep 1 && ros2 run telem_tx telem_tx_node; bash'" &
fi

echo ""
echo "==========================================="
echo "  All 5 terminals launched!"
echo "==========================================="
echo ""
echo "Total startup time: $((SITL_DELAY + MAVROS_DELAY + KML_DELAY + NAV_DELAY)) seconds"
echo ""
echo "Expected behavior:"
echo "  1. SITL starts at Kerala (${HOME_LAT}, ${HOME_LON})"
echo "  2. MAVROS connects to SITL"
echo "  3. KML Planner publishes waypoints"
echo "  4. Navigation arms and takes off"
echo "  5. Detection pipeline runs (camera + detection + telem tx)"
echo ""
echo "To inject test image during flight:"
echo "  ros2 run image_capture publish_test_image --image ${MISSIONS_FOLDER}/Yellow.png"
echo ""
echo "Monitor geotags:"
echo "  ros2 topic echo /drone1/disease_geotag"
echo ""
