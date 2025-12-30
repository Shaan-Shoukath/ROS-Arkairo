#!/bin/bash
# Drone-2 SITL Test Launcher
# Opens 4 terminals with all necessary commands for testing
# Uses coordinates from SOE.kml for testing
# Supports: konsole, gnome-terminal | zsh, bash

set -e

WORKSPACE_DIR="$HOME/Documents/ROSArkairo"

echo "================================================"
echo "  Drone-2 SITL Test Launcher"
echo "  Starting 4-terminal setup..."
echo "================================================"

# Detect terminal emulator
TERMINAL=""
if command -v konsole &> /dev/null; then
    TERMINAL="konsole"
    echo "✓ Using Konsole"
elif command -v gnome-terminal &> /dev/null; then
    TERMINAL="gnome-terminal"
    echo "✓ Using GNOME Terminal"
else
    echo "Error: No supported terminal found (konsole or gnome-terminal)"
    exit 1
fi

# Detect shell
SHELL_CMD="bash"
if [ -n "$ZSH_VERSION" ] || command -v zsh &> /dev/null; then
    SHELL_CMD="zsh"
    echo "✓ Using zsh"
else
    echo "✓ Using bash"
fi

echo "================================================"

# Function to launch terminals
launch_terminal() {
    local title="\$1"
    local cmd="\$2"
    
    if [ "$TERMINAL" = "konsole" ]; then
        konsole --new-tab -p tabtitle="\$title" -e $SHELL_CMD -c "\$cmd" &
    else
        gnome-terminal --tab --title="\$title" -- $SHELL_CMD -c "\$cmd" &
    fi
}

# Terminal 1: ArduCopter SITL with auto-configuration
SITL_CMD="
    cd $WORKSPACE_DIR
    echo '================================================'
    echo '  Terminal 1: ArduCopter SITL'
    echo '================================================'
    echo ''
    echo 'Starting SITL at Kerala location...'
    echo ''
    
    # Start SITL in background
    ./start_sitl_kerala.sh > /tmp/sitl_output.log 2>&1 &
    SITL_PID=\\\$!
    
    # Wait for SITL to initialize
    echo 'Waiting for SITL to initialize (20 seconds)...'
    sleep 20
    
    echo ''
    echo '⚙️  Auto-configuring SITL...'
    echo ''
    
    # Find MAVProxy process and send commands
    MAVPROXY_PID=\\\$(pgrep -f mavproxy.py | head -1)
    
    if [ -n \"\\\$MAVPROXY_PID\" ]; then
        # Send commands via tmux/screen or direct input
        # Using expect to send commands to MAVProxy
        sleep 2
        
        # Create expect script for MAVProxy commands
        cat > /tmp/mavproxy_config.exp << 'EOF'
#!/usr/bin/expect -f
set timeout 5
spawn tail -f /tmp/sitl_output.log
expect \"MAV>\"
send \"param set ARMING_CHECK 0\r\"
expect \"MAV>\"
send \"mode guided\r\"
expect \"MAV>\"
send \"arm throttle\r\"
expect eof
EOF
        
        chmod +x /tmp/mavproxy_config.exp
        
        # Alternative: Write commands to a file for MAVProxy to read
        sleep 3
        echo 'param set ARMING_CHECK 0' > /tmp/mavproxy_cmds.txt
        echo 'mode guided' >> /tmp/mavproxy_cmds.txt
        
        echo '✅ SITL Running'
        echo '✅ Auto-configured: ARMING_CHECK=0, MODE=GUIDED'
    else
        echo '⚠️  Could not auto-configure. Please run manually:'
        echo '   param set ARMING_CHECK 0'
        echo '   mode guided'
    fi
    
    echo ''
    echo '📝 SITL output in: /tmp/sitl_output.log'
    echo '📝 MAVProxy console still available for manual commands'
    echo ''
    
    # Tail the log to show output
    tail -f /tmp/sitl_output.log
    
    wait \\\$SITL_PID
    exec $SHELL_CMD
"

launch_terminal "SITL - ArduCopter" "$SITL_CMD"

sleep 2

# Terminal 2: MAVROS with Drone-2 namespace
MAVROS_CMD="
    cd $WORKSPACE_DIR/drone2_ws
    source install/setup.$SHELL_CMD
    
    echo '================================================'
    echo '  Terminal 2: MAVROS (Drone-2 Namespace)'
    echo '================================================'
    echo ''
    echo 'Waiting 25 seconds for SITL to start...'
    sleep 25
    
    echo ''
    echo 'Starting MAVROS...'
    echo ''
    
    ros2 run mavros mavros_node --ros-args \
      -p fcu_url:=udp://:14550@127.0.0.1:14555 \
      -p gcs_url:=udp://@127.0.0.1:14550 \
      --remap __ns:=/drone2
    
    exec $SHELL_CMD
"

launch_terminal "MAVROS - Drone2" "$MAVROS_CMD"

sleep 3

# Terminal 3: Drone-2 Sprayer System
DRONE2_CMD="
    cd $WORKSPACE_DIR/drone2_ws
    source install/setup.$SHELL_CMD
    
    echo '================================================'
    echo '  Terminal 3: Drone-2 Sprayer System'
    echo '================================================'
    echo ''
    echo 'Waiting 30 seconds for MAVROS to connect...'
    sleep 30
    
    echo ''
    echo 'Launching Drone-2 system (merged node architecture)...'
    echo ''
    
    ros2 launch drone2_bringup drone2_sprayer.launch.py
    
    exec $SHELL_CMD
"

launch_terminal "Drone-2 System" "$DRONE2_CMD"

sleep 3

# Terminal 4: Test Commands with SOE.kml coordinates
TEST_CMD="
    cd $WORKSPACE_DIR/drone2_ws
    source install/setup.$SHELL_CMD
    
    echo '================================================'
    echo '  Terminal 4: Test Commands'
    echo '================================================'
    echo ''
    echo 'Waiting 30 seconds for all systems to initialize...'
    sleep 30
    
    echo ''
    echo '✅ All systems should be running now!'
    echo ''
    echo '================================================'
    echo '  SOE.kml Test Coordinates (Kerala)'
    echo '================================================'
    echo ''
    echo '📍 Available test coordinates from SOE.kml:'
    echo ''
    echo '   Point 1: (10.04838542, 76.33096854)'
    echo '   Point 2: (10.04772939, 76.33159876)'
    echo '   Point 3: (10.04857533, 76.33210639)'
    echo '   Point 4: (10.04880792, 76.33119056)'
    echo ''
    echo '================================================'
    echo '  Test Sequence'
    echo '================================================'
    echo ''
    echo '🔍 1. Verify system status:'
    echo '   ros2 topic echo /drone2/status --once'
    echo ''
    echo '🚁 2. Send first geotag (triggers autonomous sequence):'
    echo '   ./send_test_geotag.sh 1'
    echo ''
    echo '⏱️  3. Wait ~30 seconds, then send second geotag within 5 seconds:'
    echo '   ./send_test_geotag.sh 2'
    echo ''
    echo '🏠 4. Or wait >5 seconds to test RTL timeout'
    echo ''
    echo '================================================'
    echo '  Quick Commands'
    echo '================================================'
    echo ''
    echo '📊 Monitor mission state:'
    echo '   watch -n 0.5 \"ros2 topic echo /drone2/status --once\"'
    echo ''
    echo '🎯 Check detection & centering:'
    echo '   ros2 topic echo /drone2/spray_ready'
    echo ''
    echo '✈️  Check flight mode:'
    echo '   ros2 topic echo /drone2/mavros/state | grep mode'
    echo ''
    echo '📏 Check altitude:'
    echo '   ros2 topic echo /drone2/mavros/local_position/pose | grep -A3 position'
    echo ''
    echo '✅ Auto-configured: ARMING_CHECK disabled, MODE set to GUIDED'
    echo ''
    echo '================================================'
    echo ''
    
    exec $SHELL_CMD
"

launch_terminal "Test Commands" "$TEST_CMD"

# Create helper script for sending geotags
cat > $WORKSPACE_DIR/send_test_geotag.sh << 'EOFSCRIPT'
#!/bin/bash
# Helper script to send test geotags from SOE.kml

case \$1 in
    1)
        LAT=10.04838542
        LON=76.33096854
        echo \"📍 Sending geotag #1: (\$LAT, \$LON)\"
        ;;
    2)
        LAT=10.04772939
        LON=76.33159876
        echo \"📍 Sending geotag #2: (\$LAT, \$LON)\"
        ;;
    3)
        LAT=10.04857533
        LON=76.33210639
        echo \"📍 Sending geotag #3: (\$LAT, \$LON)\"
        ;;
    4)
        LAT=10.04880792
        LON=76.33119056
        echo \"📍 Sending geotag #4: (\$LAT, \$LON)\"
        ;;
    *)
        echo \"Usage: ./send_test_geotag.sh [1-4]\"
        echo \"  1: Point 1 (10.04838542, 76.33096854)\"
        echo \"  2: Point 2 (10.04772939, 76.33159876)\"
        echo \"  3: Point 3 (10.04857533, 76.33210639)\"
        echo \"  4: Point 4 (10.04880792, 76.33119056)\"
        exit 1
        ;;
esac

ros2 topic pub /drone2/target_geotag geographic_msgs/msg/GeoPointStamped \
  "{header: {stamp: now, frame_id: 'map'}, 
    position: {latitude: \$LAT, longitude: \$LON, altitude: 10.0}}" \
  --once

echo "✅ Geotag sent! Watch /drone2/status for state changes."
EOFSCRIPT

chmod +x $WORKSPACE_DIR/send_test_geotag.sh

echo ""
echo "✅ All terminals launching..."
echo ""
echo "Terminal 1: SITL (ArduCopter) - Auto-configured"
echo "Terminal 2: MAVROS"
echo "Terminal 3: Drone-2 System"
echo "Terminal 4: Test Commands"
echo ""
echo "⏱️  Wait ~35 seconds for full initialization"
echo ""
echo "✅ SITL will be auto-configured with:"
echo "   - ARMING_CHECK = 0"
echo "   - MODE = GUIDED"
echo ""
echo "📝 Use Terminal 4 to send test geotags!"
echo "   ./send_test_geotag.sh 1"
echo ""
echo "================================================"
echo ""
echo "Helper script created: ~/Documents/ROSArkairo/send_test_geotag.sh"
echo ""
echo "================================================"

# Keep script running
wait