#!/bin/bash

# =============================================================================
# Multi-Drone SITL Test Launcher
# =============================================================================
# Launches BOTH Drone-1 and Drone-2 simultaneously for full system testing
# Opens 8 terminals with proper port separation and namespacing
# =============================================================================

set -e

echo "=========================================="
echo "  Multi-Drone SITL Test Launcher"
echo "  Testing Full System Integration"
echo "=========================================="
echo ""

# Configuration
WORKSPACE_ROOT="/home/pacman/Drone/ROS-Arkairo"
DRONE1_WS="${WORKSPACE_ROOT}/drone1_ws"
DRONE2_WS="${WORKSPACE_ROOT}/drone2_ws"
VENV_PATH="$HOME/venv-ardupilot/bin/activate"

# Detect terminal emulator
if command -v gnome-terminal &> /dev/null; then
    TERMINAL="gnome-terminal"
elif command -v konsole &> /dev/null; then
    TERMINAL="konsole"
elif command -v xterm &> /dev/null; then
    TERMINAL="xterm"
else
    echo "❌ No supported terminal emulator found!"
    echo "   Please install gnome-terminal, konsole, or xterm"
    exit 1
fi

echo "✓ Using terminal: $TERMINAL"
echo ""
echo "This will launch 8 terminals:"
echo "  1. Drone-1 SITL (Instance 0, Port 14551)"
echo "  2. Drone-2 SITL (Instance 1, Port 14561)"
echo "  3. Drone-1 MAVROS (namespace: /drone1)"
echo "  4. Drone-2 MAVROS (namespace: /drone2)"
echo "  5. Drone-1 ROS Nodes (KML, Navigation, Detection, Telemetry)"
echo "  6. Drone-2 ROS Nodes (Telemetry RX, Navigation, Detection, Sprayer)"
echo "  7. Monitoring Terminal (Status, Topics, Logs)"
echo "  8. Test Commands Terminal"
echo ""
read -p "Press Enter to start (Ctrl+C to cancel)..."
echo ""

# ============================================================================
# Terminal 1: Drone-1 SITL (Instance 0)
# ============================================================================
echo "[1/8] Launching Drone-1 SITL (Instance 0)..."

if [[ "$TERMINAL" == "gnome-terminal" ]]; then
    gnome-terminal --title="Drone-1 SITL" -- bash -c "
        source ${VENV_PATH}
        echo '=========================================='
        echo '  Drone-1 SITL (Instance 0)'
        echo '=========================================='
        echo ''
        echo 'Starting ArduCopter SITL...'
        echo 'Location: Kerala (10.0483, 76.331)'
        echo 'Ports: 5760 (SITL) → 14551 (MAVLink out)'
        echo ''
        
        cd ~/ardupilot/ArduCopter
        sim_vehicle.py -v ArduCopter -f quad \
            --console \
            --map \
            -I 0 \
            --out=udp:127.0.0.1:14551 \
            --no-rebuild \
            --custom-location=10.0483,76.331,0,0
        
        exec bash
    " &
elif [[ "$TERMINAL" == "konsole" ]]; then
    konsole --new-tab -p tabtitle="Drone-1 SITL" -e bash -c "
        source ${VENV_PATH}
        cd ~/ardupilot/ArduCopter
        sim_vehicle.py -v ArduCopter -f quad --console --map -I 0 --out=udp:127.0.0.1:14551 --no-rebuild --custom-location=10.0483,76.331,0,0
        exec bash
    " &
fi

sleep 2

# ============================================================================
# Terminal 2: Drone-2 SITL (Instance 1)
# ============================================================================
echo "[2/8] Launching Drone-2 SITL (Instance 1)..."

if [[ "$TERMINAL" == "gnome-terminal" ]]; then
    gnome-terminal --title="Drone-2 SITL" -- bash -c "
        source ${VENV_PATH}
        echo '=========================================='
        echo '  Drone-2 SITL (Instance 1)'
        echo '=========================================='
        echo ''
        echo 'Starting ArduCopter SITL...'
        echo 'Location: Kerala (10.0483, 76.331) - 5m offset'
        echo 'Ports: 5770 (SITL) → 14561 (MAVLink out)'
        echo ''
        
        cd ~/ardupilot/ArduCopter
        sim_vehicle.py -v ArduCopter -f quad \
            --console \
            --map \
            -I 1 \
            --out=udp:127.0.0.1:14561 \
            --no-rebuild \
            --custom-location=10.0483,76.331,0,0
        
        exec bash
    " &
elif [[ "$TERMINAL" == "konsole" ]]; then
    konsole --new-tab -p tabtitle="Drone-2 SITL" -e bash -c "
        source ${VENV_PATH}
        cd ~/ardupilot/ArduCopter
        sim_vehicle.py -v ArduCopter -f quad --console --map -I 1 --out=udp:127.0.0.1:14561 --no-rebuild --custom-location=10.0483,76.331,0,0
        exec bash
    " &
fi

echo "   Waiting 30 seconds for both SITL instances to initialize..."
sleep 30

# ============================================================================
# Terminal 3: Drone-1 MAVROS
# ============================================================================
echo "[3/8] Launching Drone-1 MAVROS..."

if [[ "$TERMINAL" == "gnome-terminal" ]]; then
    gnome-terminal --title="Drone-1 MAVROS" -- bash -c "
        source /opt/ros/jazzy/setup.bash
        echo '=========================================='
        echo '  Drone-1 MAVROS'
        echo '=========================================='
        echo ''
        echo 'Namespace: /drone1'
        echo 'FCU URL: udp://:14551@127.0.0.1:14550'
        echo ''
        
        ros2 launch mavros apm.launch \
            fcu_url:=udp://:14551@127.0.0.1:14550 \
            namespace:=drone1
        
        exec bash
    " &
elif [[ "$TERMINAL" == "konsole" ]]; then
    konsole --new-tab -p tabtitle="Drone-1 MAVROS" -e bash -c "
        source /opt/ros/jazzy/setup.bash
        ros2 launch mavros apm.launch fcu_url:=udp://:14551@127.0.0.1:14550 namespace:=drone1
        exec bash
    " &
fi

sleep 3

# ============================================================================
# Terminal 4: Drone-2 MAVROS
# ============================================================================
echo "[4/8] Launching Drone-2 MAVROS..."

if [[ "$TERMINAL" == "gnome-terminal" ]]; then
    gnome-terminal --title="Drone-2 MAVROS" -- bash -c "
        source /opt/ros/jazzy/setup.bash
        echo '=========================================='
        echo '  Drone-2 MAVROS'
        echo '=========================================='
        echo ''
        echo 'Namespace: /drone2'
        echo 'FCU URL: udp://:14561@127.0.0.1:14560'
        echo ''
        
        ros2 launch mavros apm.launch \
            fcu_url:=udp://:14561@127.0.0.1:14560 \
            namespace:=drone2
        
        exec bash
    " &
elif [[ "$TERMINAL" == "konsole" ]]; then
    konsole --new-tab -p tabtitle="Drone-2 MAVROS" -e bash -c "
        source /opt/ros/jazzy/setup.bash
        ros2 launch mavros apm.launch fcu_url:=udp://:14561@127.0.0.1:14560 namespace:=drone2
        exec bash
    " &
fi

echo "   Waiting 15 seconds for MAVROS connections..."
sleep 15

# ============================================================================
# Terminal 5: Drone-1 System (All Nodes)
# ============================================================================
echo "[5/8] Launching Drone-1 System..."

if [[ "$TERMINAL" == "gnome-terminal" ]]; then
    gnome-terminal --title="Drone-1 System" -- bash -c "
        cd ${DRONE1_WS}
        source /opt/ros/jazzy/setup.bash
        source install/setup.bash
        echo '=========================================='
        echo '  Drone-1 System (Survey Drone)'
        echo '=========================================='
        echo ''
        echo 'Launching nodes:'
        echo '  - KML Lane Planner'
        echo '  - Navigation Node'
        echo '  - Image Capture'
        echo '  - Detection & Geotag'
        echo '  - Telemetry TX'
        echo ''
        
        # Check if launch file exists
        if [ -f \"src/drone1_bringup/launch/drone1_survey.launch.py\" ]; then
            echo 'Using launch file...'
            ros2 launch drone1_bringup drone1_survey.launch.py
        else
            echo 'Launch file not found, running nodes individually...'
            echo ''
            
            # Start KML Planner
            ros2 run kml_lane_planner kml_lane_planner_node --ros-args \
                -p missions_folder:=${DRONE1_WS}/missions \
                -p kml_filename:=SOE.kml \
                -p lane_spacing_m:=10.0 \
                -p altitude_m:=10.0 \
                -p enable_republish:=false &
            
            sleep 3
            
            # Start Navigation
            ros2 run drone1_navigation drone1_navigation_node --ros-args \
                -p takeoff_altitude_m:=10.0 \
                -p navigation_altitude_m:=10.0 \
                -p path_lookahead_m:=12.0 \
                -p fcu_timeout_sec:=60.0 &
            
            sleep 2
            
            # Start Detection (if available)
            if ros2 pkg list | grep -q detection_and_geotag; then
                ros2 run detection_and_geotag detection_test_node --ros-args \
                    --params-file src/detection_and_geotag/config/test_params.yaml &
            fi
            
            # Start Telemetry TX (if available)
            if ros2 pkg list | grep -q telem_tx; then
                ros2 run telem_tx telem_tx_node &
            fi
            
            wait
        fi
        
        exec bash
    " &
elif [[ "$TERMINAL" == "konsole" ]]; then
    konsole --new-tab -p tabtitle="Drone-1 System" -e bash -c "
        cd ${DRONE1_WS}
        source /opt/ros/jazzy/setup.bash
        source install/setup.bash
        ros2 launch drone1_bringup drone1_survey.launch.py || (
            ros2 run kml_lane_planner kml_lane_planner_node &
            ros2 run drone1_navigation drone1_navigation_node &
            wait
        )
        exec bash
    " &
fi

sleep 5

# ============================================================================
# Terminal 6: Drone-2 System (All Nodes)
# ============================================================================
echo "[6/8] Launching Drone-2 System..."

if [[ "$TERMINAL" == "gnome-terminal" ]]; then
    gnome-terminal --title="Drone-2 System" -- bash -c "
        cd ${DRONE2_WS}
        source /opt/ros/jazzy/setup.bash
        source install/setup.bash
        echo '=========================================='
        echo '  Drone-2 System (Sprayer Drone)'
        echo '=========================================='
        echo ''
        echo 'Launching nodes:'
        echo '  - Telemetry RX'
        echo '  - Navigation Node'
        echo '  - Detection & Centering'
        echo '  - Sprayer Control'
        echo ''
        echo 'Waiting for first geotag from Drone-1...'
        echo ''
        
        ros2 launch drone2_bringup drone2_sprayer.launch.py
        
        exec bash
    " &
elif [[ "$TERMINAL" == "konsole" ]]; then
    konsole --new-tab -p tabtitle="Drone-2 System" -e bash -c "
        cd ${DRONE2_WS}
        source /opt/ros/jazzy/setup.bash
        source install/setup.bash
        ros2 launch drone2_bringup drone2_sprayer.launch.py
        exec bash
    " &
fi

sleep 5

# ============================================================================
# Terminal 7: Monitoring
# ============================================================================
echo "[7/8] Launching Monitoring Terminal..."

if [[ "$TERMINAL" == "gnome-terminal" ]]; then
    gnome-terminal --title="System Monitor" -- bash -c "
        source /opt/ros/jazzy/setup.bash
        echo '=========================================='
        echo '  System Monitoring Dashboard'
        echo '=========================================='
        echo ''
        echo 'Waiting 5 seconds for all nodes to start...'
        sleep 5
        echo ''
        echo '✅ All systems launched!'
        echo ''
        echo '================================================'
        echo '  Quick Monitoring Commands'
        echo '================================================'
        echo ''
        echo '📊 Drone-1 Status:'
        echo '   ros2 topic echo /drone1/navigation_status'
        echo ''
        echo '📊 Drone-2 Status:'
        echo '   ros2 topic echo /drone2/status'
        echo ''
        echo '📡 Geotags (Drone-1 → Drone-2):'
        echo '   ros2 topic echo /drone1/disease_geotag'
        echo ''
        echo '🎯 Target Received (Drone-2):'
        echo '   ros2 topic echo /drone2/target_geotag'
        echo ''
        echo '✈️  Drone-1 Position:'
        echo '   ros2 topic echo /drone1/mavros/local_position/pose | grep -A3 position'
        echo ''
        echo '✈️  Drone-2 Position:'
        echo '   ros2 topic echo /drone2/mavros/local_position/pose | grep -A3 position'
        echo ''
        echo '🔍 List all nodes:'
        echo '   ros2 node list'
        echo ''
        echo '================================================'
        echo ''
        
        exec bash
    " &
elif [[ "$TERMINAL" == "konsole" ]]; then
    konsole --new-tab -p tabtitle="Monitor" -e bash -c "
        source /opt/ros/jazzy/setup.bash
        sleep 5
        echo 'Monitoring terminal ready'
        exec bash
    " &
fi

sleep 2

# ============================================================================
# Terminal 8: Test Commands
# ============================================================================
echo "[8/8] Launching Test Commands Terminal..."

if [[ "$TERMINAL" == "gnome-terminal" ]]; then
    gnome-terminal --title="Test Commands" -- bash -c "
        cd ${WORKSPACE_ROOT}
        source /opt/ros/jazzy/setup.bash
        echo '=========================================='
        echo '  Test Commands & Emergency Controls'
        echo '=========================================='
        echo ''
        echo 'Waiting 10 seconds for system stabilization...'
        sleep 10
        echo ''
        echo '✅ System ready for testing!'
        echo ''
        echo '================================================'
        echo '  Expected Mission Flow'
        echo '================================================'
        echo ''
        echo '1. Drone-1 detects SOE.kml → Arms → Takeoff → Survey'
        echo '2. Drone-1 flies lawnmower pattern at 10m altitude'
        echo '3. Drone-1 detects diseases → Sends geotags via telemetry'
        echo '4. Drone-2 receives first geotag → Arms → Takeoff'
        echo '5. Drone-2 navigates to target → Detects → Centers → Sprays'
        echo '6. Drone-2 waits for next geotag (15s timeout)'
        echo '7. Repeat steps 3-6 until survey complete'
        echo '8. Both drones RTL when mission complete'
        echo ''
        echo '================================================'
        echo '  Manual Test Commands'
        echo '================================================'
        echo ''
        echo '🧪 Send manual geotag to Drone-2:'
        echo '   ros2 topic pub /drone2/target_geotag geographic_msgs/msg/GeoPointStamped \\'
        echo '     \"{header: {stamp: now, frame_id: map}, \\'
        echo '       position: {latitude: 10.0484, longitude: 76.3312, altitude: 10.0}}\" --once'
        echo ''
        echo '================================================'
        echo '  Emergency Controls'
        echo '================================================'
        echo ''
        echo '🛑 Emergency RTL (Drone-1):'
        echo '   ros2 service call /drone1/mavros/set_mode mavros_msgs/srv/SetMode \\'
        echo '     \"{custom_mode: RTL}\"'
        echo ''
        echo '🛑 Emergency RTL (Drone-2):'
        echo '   ros2 service call /drone2/mavros/set_mode mavros_msgs/srv/SetMode \\'
        echo '     \"{custom_mode: RTL}\"'
        echo ''
        echo '🛑 Emergency LAND (Both):'
        echo '   ros2 service call /drone1/mavros/set_mode mavros_msgs/srv/SetMode \\'
        echo '     \"{custom_mode: LAND}\"'
        echo '   ros2 service call /drone2/mavros/set_mode mavros_msgs/srv/SetMode \\'
        echo '     \"{custom_mode: LAND}\"'
        echo ''
        echo '================================================'
        echo ''
        
        exec bash
    " &
elif [[ "$TERMINAL" == "konsole" ]]; then
    konsole --new-tab -p tabtitle="Test Commands" -e bash -c "
        cd ${WORKSPACE_ROOT}
        source /opt/ros/jazzy/setup.bash
        sleep 10
        echo 'Test terminal ready'
        exec bash
    " &
fi

echo ""
echo "=========================================="
echo "  ✅ All 8 terminals launched!"
echo "=========================================="
echo ""
echo "Total initialization time: ~50 seconds"
echo ""
echo "📋 System Layout:"
echo "   Terminal 1-2: SITL instances (MAVProxy consoles)"
echo "   Terminal 3-4: MAVROS connections"
echo "   Terminal 5-6: ROS2 node systems"
echo "   Terminal 7:   Monitoring dashboard"
echo "   Terminal 8:   Test commands & emergency controls"
echo ""
echo "🎯 What to watch:"
echo "   - Drone-1 should auto-arm and start survey"
echo "   - Drone-2 waits in IDLE until first geotag"
echo "   - Both drones operate autonomously"
echo ""
echo "⚠️  Remember: RC transmitter override works in SITL too!"
echo "   Use MAVProxy console: 'mode LOITER' to take manual control"
echo ""
echo "================================================"
echo ""

# Keep script running
wait
