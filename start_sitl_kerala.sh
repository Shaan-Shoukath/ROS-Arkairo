#!/bin/zsh

# =============================================================================
# ArduPilot SITL for Drone-1 Testing - Kerala Location
# =============================================================================
# Location matches KML polygon in drone1_ws/missions/SOE.kml
# Home: 10.0483°N, 76.331°E (School of Engineering area)
# =============================================================================

echo "=========================================="
echo "  ArduPilot SITL - Kerala Location"
echo "=========================================="
echo "Home: 10.0483°N, 76.331°E"
echo ""

# Activate ArduPilot virtual environment
echo "Activating ArduPilot venv..."
source ~/venv-ardupilot/bin/activate

# Create a params file to disable auto-disarm for testing
PARAMS_FILE="/tmp/sitl_test_params.parm"
cat > $PARAMS_FILE << 'PARAMS'
DISARM_DELAY 0
ARMING_CHECK 0
FS_THR_ENABLE 0
PARAMS
echo "Created test parameters file: $PARAMS_FILE"

echo ""
echo "After SITL starts, run in separate terminals:"
echo ""
echo "Terminal 2 - MAVROS:"
echo "  source /opt/ros/jazzy/setup.zsh"
echo "  ros2 launch mavros apm.launch fcu_url:=udp://:14551@127.0.0.1:14550"
echo ""
echo "Terminal 3 - KML Planner:"
echo "  cd ~/Documents/ROSArkairo/drone1_ws"
echo "  source /opt/ros/jazzy/setup.zsh && source install/setup.zsh"
echo "  ros2 run kml_lane_planner kml_lane_planner_node --ros-args \\"
echo "    -p missions_folder:=/home/shaan-shoukath/Documents/ROSArkairo/drone1_ws/missions \\"
echo "    -p kml_filename:=SOE.kml \\"
echo "    -p lane_spacing_m:=10.0 \\"
echo "    -p altitude_m:=10.0"
echo ""
echo "Terminal 4 - Navigation:"
echo "  cd ~/Documents/ROSArkairo/drone1_ws"
echo "  source /opt/ros/jazzy/setup.zsh && source install/setup.zsh"
echo "  ros2 run drone1_navigation drone1_navigation_node --ros-args \\"
echo "    -p takeoff_altitude_m:=10.0 \\"
echo "    -p navigation_altitude_m:=10.0"
echo ""
echo "=========================================="
echo ""
echo "SITL will start with:"
echo "  - DISARM_DELAY = 0 (no auto-disarm)"
echo "  - ARMING_CHECK = 0 (skip prearm checks)"
echo ""

# Start SITL with:
# - Home at Kerala KML location
# - UDP output for MAVROS on port 14551
# - Console and map for visualization
# - Extra params file to disable auto-disarm
sim_vehicle.py -v ArduCopter -f quad \
    --console \
    --map \
    --out=udp:127.0.0.1:14551 \
    --no-rebuild \
    --add-param-file=$PARAMS_FILE \
    --custom-location=10.0483,76.331,0,0
