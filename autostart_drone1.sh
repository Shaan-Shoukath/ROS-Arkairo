#!/usr/bin/env bash
# =============================================================================
# Drone-1 Autostart Script
# =============================================================================
# Author: Shaan Shoukath
# Hardware: Raspberry Pi 5 + Cube Orange+ + Pi Camera 3
# Purpose: Auto-launch all Drone-1 nodes on boot
# =============================================================================

set -e

# ---------- CONFIGURATION ----------
USER_HOME="/home/arkairo1"
LOG_DIR="$USER_HOME/logs"
WORKSPACE="$USER_HOME/drone1_ws"

# Create log directory
mkdir -p $LOG_DIR

LOG="$LOG_DIR/drone1_boot.log"
echo "=== Drone-1 boot $(date) ===" >> $LOG

# ---------- ENVIRONMENT ----------
source /opt/ros/jazzy/setup.bash
source $WORKSPACE/install/setup.bash

echo "ROS=$ROS_DISTRO" >> $LOG
echo "WORKSPACE=$WORKSPACE" >> $LOG
which python >> $LOG

# ---------- BOOT DELAY ----------
# Wait for system to fully initialize (serial ports, network, etc.)
echo "[$(date +%H:%M:%S)] Waiting 30 seconds for system initialization..." >> $LOG
sleep 30

# ---------- 1. MAVROS ----------
echo "[$(date +%H:%M:%S)] Starting MAVROS..." >> $LOG
sleep 10

ros2 launch mavros apm.launch \
  fcu_url:=serial:///dev/ttyAMA0:9600\
  >> $LOG_DIR/mavros.log 2>&1 &

# Wait for FCU connection
echo "[$(date +%H:%M:%S)] Waiting for FCU connection..." >> $LOG
sleep 20

# ---------- 2. KML Lane Planner ----------
echo "[$(date +%H:%M:%S)] Starting KML Lane Planner..." >> $LOG
ros2 run kml_lane_planner kml_lane_planner_node \
  --ros-args -p require_gps_home:=false \
  >> $LOG_DIR/kml_planner.log 2>&1 &

sleep 5

# ---------- 3. Navigation Node ----------
echo "[$(date +%H:%M:%S)] Starting Navigation Node..." >> $LOG
ros2 run drone1_navigation drone1_navigation_node \
  >> $LOG_DIR/navigation.log 2>&1 &

sleep 5

# ---------- 4. Image Capture (Pi Camera 3) ----------
echo "[$(date +%H:%M:%S)] Starting Image Capture (Pi Camera 3)..." >> $LOG
ros2 run image_capture image_capture_node \
  --ros-args -p use_sim:=false \
  >> $LOG_DIR/image_capture.log 2>&1 &

sleep 3

# ---------- 5. Detection & Geotag ----------
echo "[$(date +%H:%M:%S)] Starting Detection & Geotag..." >> $LOG
ros2 run detection_and_geotag detection_and_geotag_node \
  >> $LOG_DIR/detection.log 2>&1 &

sleep 3

# ---------- 6. Telemetry TX ----------
echo "[$(date +%H:%M:%S)] Starting Telemetry TX..." >> $LOG
ros2 run telem_tx telem_tx_node \
  >> $LOG_DIR/telem_tx.log 2>&1 &

echo "[$(date +%H:%M:%S)] All nodes started!" >> $LOG
echo "=== Drone-1 ready ===" >> $LOG

# Keep script running to maintain child processes
wait
