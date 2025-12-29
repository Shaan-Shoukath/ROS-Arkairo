#!/usr/bin/env python3
# Copyright 2024 Shaan Shoukath
# SPDX-License-Identifier: Apache-2.0

"""
Drone-1 Autonomous Flight Controller
=====================================
Hardware: Cube Orange+ | Companion: Raspberry Pi 5
ROS: Jazzy | Middleware: MAVROS | Mode: GUIDED ONLY

Architecture:
  KML Planner → LaneSegmentArray → This Node → MAVROS → MAVLink → ArduCopter

State Machine:
  IDLE → WAIT_FOR_FCU → WAIT_FOR_MISSION → SET_GUIDED → ARM →
  TAKEOFF_CMD → WAIT_FOR_TAKEOFF → NAVIGATE → MISSION_COMPLETE → RTL

Critical Design Rules:
  ✅ MAV_CMD_NAV_TAKEOFF is the ONLY way to initiate flight
  ✅ Position setpoints stream at 10Hz starting from ARM state (ArduPilot requirement)
  ✅ ARM happens exactly once
  ✅ TAKEOFF happens exactly once
  ✅ Detection enabled ONLY during NAVIGATE
  ✅ Throttle/PWM controlled exclusively by ArduPilot
  ✅ No RC override, no velocity-based takeoff, no AUTO mode

Subscribers:
  /mission/lane_segments               - Waypoints from KML planner
  /mavros/state                        - FCU connection & mode
  /mavros/global_position/relative_alt - Takeoff altitude reference
  /mavros/local_position/pose          - Navigation position reference

Publishers:
  /mavros/setpoint_position/local      - Position setpoints (10Hz continuous)
  /drone1/next_waypoint                - Current target for monitoring
  /drone1/navigation_status            - FSM state
  /drone1/detection_enable             - Detection control
"""

import math
import os
from datetime import datetime
from enum import Enum, auto
from typing import Optional, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String, Bool, Float64
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, ParamSetV2
from rcl_interfaces.msg import ParameterValue, ParameterType
from drone1_msgs.msg import LaneSegmentArray


class FlightState(Enum):
    """Flight state machine - progresses linearly, no loops."""
    IDLE = auto()
    WAIT_FOR_FCU = auto()
    WAIT_FOR_MISSION = auto()
    SET_GUIDED = auto()
    ARM = auto()
    TAKEOFF_CMD = auto()
    WAIT_FOR_TAKEOFF = auto()
    NAVIGATE = auto()
    MISSION_COMPLETE = auto()
    RTL = auto()
    ERROR = auto()


class Drone1FlightController(Node):
    """
    Autonomous flight controller for Drone-1 survey missions.

    Designed for real hardware (Cube Orange+ / Raspberry Pi 5) and SITL testing.
    Uses GUIDED mode exclusively with position-based control.
    """

    def __init__(self):
        super().__init__('drone1_navigation_node')

        # ====================================================================
        # PARAMETERS
        # ====================================================================
        self.declare_parameter('takeoff_altitude_m', 50.0)
        self.declare_parameter('navigation_altitude_m', 50.0)
        self.declare_parameter('waypoint_arrival_radius_m', 3.0)
        self.declare_parameter('path_lookahead_m', 12.0)
        self.declare_parameter('path_max_target_step_m', 25.0)
        self.declare_parameter('path_end_radius_m', 5.0)
        self.declare_parameter('setpoint_rate_hz', 10.0)
        self.declare_parameter('takeoff_timeout_sec', 90.0)
        self.declare_parameter('fcu_timeout_sec', 30.0)

        self.takeoff_altitude = self.get_parameter('takeoff_altitude_m').value
        self.navigation_altitude = self.get_parameter('navigation_altitude_m').value
        self.arrival_radius = self.get_parameter('waypoint_arrival_radius_m').value
        self.path_lookahead_m = self.get_parameter('path_lookahead_m').value
        self.path_max_target_step_m = self.get_parameter('path_max_target_step_m').value
        self.path_end_radius_m = self.get_parameter('path_end_radius_m').value
        self.setpoint_rate = self.get_parameter('setpoint_rate_hz').value
        self.takeoff_timeout = self.get_parameter('takeoff_timeout_sec').value
        self.fcu_timeout = self.get_parameter('fcu_timeout_sec').value

        # ====================================================================
        # STATE VARIABLES
        # ====================================================================
        self.state = FlightState.IDLE
        self.state_entry_time = self.get_clock().now()

        # FCU state
        self.fcu_connected = False
        self.fcu_state: Optional[State] = None
        self.has_armed_once = False  # Prevent re-arming
        self.has_taken_off_once = False  # Prevent re-takeoff

        # Position tracking
        self.relative_altitude: Optional[float] = None  # Takeoff reference
        self.local_pose: Optional[PoseStamped] = None   # Navigation reference
        self.home_local_pose: Optional[PoseStamped] = None  # Set at takeoff
        self.use_local_z_for_altitude = False  # Fallback if relative_alt topic missing

        # Mission data
        self.waypoints_gps: List[Tuple[float, float, float]] = []  # (lat, lon, alt)
        self.waypoints_local: List[Tuple[float, float, float]] = []  # (x, y, z) in ENU
        self.current_waypoint_idx = 0
        self.home_gps: Optional[Tuple[float, float]] = None  # For local conversion

        # Path-following state (pure-pursuit style)
        self.path_xy: List[Tuple[float, float]] = []
        self.path_cum_s: List[float] = []
        self.path_length_m: float = 0.0
        self.path_progress_s: float = 0.0
        self.current_target_local: Optional[Tuple[float, float, float]] = None
        self._last_published_waypoint_idx: int = -1
        self._last_nav_log_sec: int = -1

        # ====================================================================
        # QOS PROFILES
        # ====================================================================
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        mission_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # ====================================================================
        # SUBSCRIBERS
        # ====================================================================
        self.create_subscription(
            LaneSegmentArray,
            '/mission/lane_segments',
            self.mission_callback,
            mission_qos
        )

        self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            sensor_qos
        )

        self.create_subscription(
            Float64,
            '/mavros/global_position/relative_alt',
            self.relative_alt_callback,
            sensor_qos
        )

        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.local_pose_callback,
            sensor_qos
        )

        # GPS subscription for home position capture
        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            sensor_qos
        )

        # ====================================================================
        # PUBLISHERS
        # ====================================================================
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        self.waypoint_pub = self.create_publisher(
            NavSatFix,
            '/drone1/next_waypoint',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/drone1/navigation_status',
            10
        )

        self.detection_pub = self.create_publisher(
            Bool,
            '/drone1/detection_enable',
            10
        )

        # ====================================================================
        # SERVICE CLIENTS
        # ====================================================================
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.param_client = self.create_client(ParamSetV2, '/mavros/param/set')
        
        # Track if params configured for SITL testing
        self.params_configured = False

        # ====================================================================
        # TIMERS
        # ====================================================================
        # FSM update at 2Hz
        self.create_timer(0.5, self.fsm_update)

        # Setpoint publishing at configured rate (10Hz default)
        self.create_timer(1.0 / self.setpoint_rate, self.publish_setpoint)

        # Status publishing at 1Hz
        self.create_timer(1.0, self.publish_status)

        # ====================================================================
        # LOGGING
        # ====================================================================
        log_dir = os.path.expanduser('~/Documents/ROSArkairo/logs')
        os.makedirs(log_dir, exist_ok=True)
        log_file = os.path.join(log_dir, 'drone1_flight_latest.log')

        self.log_file = open(log_file, 'w')
        self.log(f"Drone-1 Flight Controller initialized at {datetime.now()}")
        self.log(f"Takeoff altitude: {self.takeoff_altitude}m")
        self.log(f"Navigation altitude: {self.navigation_altitude}m")
        self.log(f"Arrival radius: {self.arrival_radius}m")

        self.get_logger().info("=" * 60)
        self.get_logger().info("Drone-1 Flight Controller - Position Control Mode")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Takeoff: {self.takeoff_altitude}m | Navigate: {self.navigation_altitude}m")
        self.get_logger().info("Waiting for mission from KML planner...")

    def log(self, message: str):
        """Write to both ROS logger and file."""
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        self.log_file.write(f"[{timestamp}] {message}\n")
        self.log_file.flush()

    # ========================================================================
    # CALLBACKS
    # ========================================================================

    def mission_callback(self, msg: LaneSegmentArray):
        """Receive mission waypoints from KML planner."""
        if self.state not in [FlightState.WAIT_FOR_MISSION, FlightState.IDLE, FlightState.WAIT_FOR_FCU]:
            self.get_logger().warn("Mission received but already in flight - ignoring")
            return

        self.waypoints_gps.clear()

        # Extract waypoints from lane segments (start_waypoint, end_waypoint pairs)
        for lane in msg.lanes:
            self.waypoints_gps.append((
                lane.start_waypoint.latitude,
                lane.start_waypoint.longitude,
                self.navigation_altitude
            ))
            self.waypoints_gps.append((
                lane.end_waypoint.latitude,
                lane.end_waypoint.longitude,
                self.navigation_altitude
            ))

        self.get_logger().info(f"Mission received: {len(self.waypoints_gps)} waypoints from {len(msg.lanes)} lanes")
        self.log(f"Mission loaded: {len(self.waypoints_gps)} waypoints at {self.navigation_altitude}m altitude")

        if len(self.waypoints_gps) > 0 and self.state == FlightState.WAIT_FOR_MISSION:
            self.transition_to(FlightState.SET_GUIDED, "Mission loaded")

    def state_callback(self, msg: State):
        """Monitor FCU connection and mode."""
        self.fcu_state = msg

        # Detect connection
        if msg.connected and not self.fcu_connected:
            self.fcu_connected = True
            self.get_logger().info("FCU connected")
            self.log("FCU connection established")

        # Detect disconnection - emergency RTL
        if not msg.connected and self.fcu_connected:
            self.get_logger().error("FCU DISCONNECTED - Emergency RTL")
            self.log("FCU connection lost - initiating RTL")
            self.transition_to(FlightState.RTL, "FCU disconnected")

        # Detect mode exit during flight - emergency RTL
        if self.state in [FlightState.ARM, FlightState.TAKEOFF_CMD,
                          FlightState.WAIT_FOR_TAKEOFF, FlightState.NAVIGATE]:
            if msg.mode != 'GUIDED':
                self.get_logger().error(f"Lost GUIDED mode (now {msg.mode}) - Emergency RTL")
                self.log(f"Mode changed from GUIDED to {msg.mode} - initiating RTL")
                self.transition_to(FlightState.RTL, "Lost GUIDED mode")

    def relative_alt_callback(self, msg: Float64):
        """Track relative altitude for takeoff detection."""
        self.relative_altitude = msg.data

    def local_pose_callback(self, msg: PoseStamped):
        """Track local position for navigation."""
        self.local_pose = msg

        # Capture home position when takeoff completes
        if self.state == FlightState.WAIT_FOR_TAKEOFF and self.home_local_pose is None:
            if self.relative_altitude and self.relative_altitude > 5.0:
                self.home_local_pose = msg
                self.get_logger().info(
                    f"Home position captured: x={msg.pose.position.x:.2f}, "
                    f"y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}"
                )
                self.log(f"Home local position: ({msg.pose.position.x:.2f}, "
                         f"{msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})")

    def gps_callback(self, msg: NavSatFix):
        """Capture GPS for coordinate conversion."""
        if self.home_gps is None and msg.latitude != 0.0:
            self.home_gps = (msg.latitude, msg.longitude)
            self.get_logger().info(f"Home GPS: ({msg.latitude:.6f}, {msg.longitude:.6f})")

    # ========================================================================
    # STATE MACHINE
    # ========================================================================

    def transition_to(self, new_state: FlightState, reason: str = ""):
        """Transition to a new state with logging."""
        old_state = self.state
        self.state = new_state
        self.state_entry_time = self.get_clock().now()

        log_msg = f"FSM: {old_state.name} → {new_state.name}"
        if reason:
            log_msg += f" ({reason})"

        self.get_logger().info(log_msg)
        self.log(log_msg)

        # State entry actions
        if new_state == FlightState.RTL:
            self.detection_pub.publish(Bool(data=False))
            self.request_rtl()
        elif new_state == FlightState.NAVIGATE:
            self.detection_pub.publish(Bool(data=True))
            self.convert_waypoints_to_local()
            self.path_progress_s = 0.0
            self.current_target_local = None
            self._last_published_waypoint_idx = -1
            self._last_nav_log_sec = -1

    def time_in_state(self) -> float:
        """Get seconds elapsed in current state."""
        return (self.get_clock().now() - self.state_entry_time).nanoseconds / 1e9

    def fsm_update(self):
        """Main state machine update (2Hz)."""

        if self.state == FlightState.IDLE:
            self.handle_idle()

        elif self.state == FlightState.WAIT_FOR_FCU:
            self.handle_wait_for_fcu()

        elif self.state == FlightState.WAIT_FOR_MISSION:
            self.handle_wait_for_mission()

        elif self.state == FlightState.SET_GUIDED:
            self.handle_set_guided()

        elif self.state == FlightState.ARM:
            self.handle_arm()

        elif self.state == FlightState.TAKEOFF_CMD:
            self.handle_takeoff_cmd()

        elif self.state == FlightState.WAIT_FOR_TAKEOFF:
            self.handle_wait_for_takeoff()

        elif self.state == FlightState.NAVIGATE:
            self.handle_navigate()

        elif self.state == FlightState.MISSION_COMPLETE:
            self.handle_mission_complete()

        elif self.state == FlightState.RTL:
            pass  # Terminal state

        elif self.state == FlightState.ERROR:
            pass  # Terminal state

    # ========================================================================
    # STATE HANDLERS
    # ========================================================================

    def handle_idle(self):
        """Initial state - wait for system ready."""
        self.transition_to(FlightState.WAIT_FOR_FCU, "System initialized")

    def handle_wait_for_fcu(self):
        """Wait for MAVROS connection to FCU."""
        if self.fcu_connected:
            self.transition_to(FlightState.WAIT_FOR_MISSION, "FCU ready")
            return

        if self.time_in_state() > self.fcu_timeout:
            self.get_logger().error("FCU connection timeout")
            self.log("FCU connection timeout exceeded")
            self.transition_to(FlightState.ERROR, "FCU timeout")

    def handle_wait_for_mission(self):
        """Wait for mission waypoints from KML planner."""
        # Mission callback handles transition
        if len(self.waypoints_gps) > 0:
            self.transition_to(FlightState.SET_GUIDED, "Mission available")

    def handle_set_guided(self):
        """Set GUIDED mode and configure SITL parameters."""
        if not self.fcu_state:
            return

        if self.fcu_state.mode == 'GUIDED':
            # Configure SITL parameters before arming (only once)
            if not self.params_configured:
                self.configure_sitl_params()
                self.params_configured = True
                return  # Wait for params to be set
            
            self.transition_to(FlightState.ARM, "GUIDED mode confirmed")
            return

        # Request GUIDED mode (rate limited)
        if self.time_in_state() < 1.0:
            return

        if self.mode_client.wait_for_service(timeout_sec=0.5):
            request = SetMode.Request()
            request.custom_mode = 'GUIDED'
            future = self.mode_client.call_async(request)
            self.get_logger().info("Requesting GUIDED mode")
            self.log("GUIDED mode request sent")

    def configure_sitl_params(self):
        """Set ArduPilot parameters for SITL testing (disable auto-disarm)."""
        if not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Param service not available - skipping SITL config")
            return
        
        # Set DISARM_DELAY to 0 (disable auto-disarm on ground)
        request = ParamSetV2.Request()
        request.param_id = 'DISARM_DELAY'
        request.value = ParameterValue()
        request.value.type = ParameterType.PARAMETER_INTEGER
        request.value.integer_value = 0
        
        future = self.param_client.call_async(request)
        self.get_logger().info("Setting DISARM_DELAY=0 for SITL testing")
        self.log("DISARM_DELAY=0 set via MAVROS")

    def handle_arm(self):
        """
        ARM state - critical for ArduPilot GUIDED mode.

        Position setpoints MUST stream before arming to prevent "Mission is stale" error.
        This state waits 3 seconds for setpoints to flow, then arms.
        """
        if not self.fcu_state:
            return

        # Verify still in GUIDED mode
        if self.fcu_state.mode != 'GUIDED':
            self.get_logger().warn(f"Not in GUIDED mode ({self.fcu_state.mode}), returning to SET_GUIDED")
            self.transition_to(FlightState.SET_GUIDED, "Lost GUIDED mode")
            return

        # Check if already armed (manual or previous attempt)
        if self.fcu_state.armed:
            if not self.has_armed_once:
                self.has_armed_once = True
            self.transition_to(FlightState.TAKEOFF_CMD, "Armed detected")
            return

        # Prevent re-arming after successful arm
        if self.has_armed_once and not self.fcu_state.armed:
            # Already armed once and now disarmed - go to RTL
            self.get_logger().error("Disarmed after arming - entering RTL")
            self.log("Unexpected disarm - entering RTL")
            self.transition_to(FlightState.RTL, "Disarmed after arm")
            return

        # Wait for setpoints to stream (3 seconds)
        if self.time_in_state() < 3.0:
            elapsed = self.time_in_state()
            if int(elapsed * 2) % 2 == 0:  # Log every 0.5s
                self.get_logger().info(f"Streaming setpoints... ({elapsed:.1f}s / 3.0s)")
            return

        # Send arming command (retry every 2 seconds until armed)
        time_since_3s = self.time_in_state() - 3.0
        # Send at 0s, 2s, 4s, 6s... after the 3s streaming period
        interval_num = int(time_since_3s / 2.0)
        time_in_interval = time_since_3s - (interval_num * 2.0)
        
        # Send command at the start of each 2s interval (first 0.6s window)
        if time_in_interval < 0.6:
            if self.arming_client.wait_for_service(timeout_sec=0.5):
                request = CommandBool.Request()
                request.value = True
                future = self.arming_client.call_async(request)
                self.get_logger().info(f"ARM command sent (attempt {interval_num + 1})")
                self.log("ARM command sent to FCU")

        # Timeout after 15 seconds of trying to arm
        if self.time_in_state() > 18.0:
            self.get_logger().error("ARM timeout - failed to arm")
            self.log("ARM timeout after 15 seconds")
            self.transition_to(FlightState.RTL, "ARM timeout")

    def handle_takeoff_cmd(self):
        """
        Initiate takeoff using MAVROS takeoff service.

        Uses MAV_CMD_NAV_TAKEOFF via `/mavros/cmd/takeoff` (CommandTOL).
        Position setpoints continue streaming in parallel for GUIDED stability.
        """
        if not self.fcu_state:
            return

        # Verify still armed
        if not self.fcu_state.armed:
            self.get_logger().error("Disarmed before takeoff")
            self.log("Unexpected disarm before takeoff - entering RTL")
            self.transition_to(FlightState.RTL, "Disarmed prematurely")
            return

        # Prevent re-takeoff
        if self.has_taken_off_once:
            self.get_logger().error("Re-takeoff blocked - safety violation")
            self.log("Re-takeoff attempt blocked - entering RTL")
            self.transition_to(FlightState.RTL, "Re-takeoff blocked")
            return

        # Wait briefly for arm to stabilize (1 second)
        if self.time_in_state() < 1.0:
            return

        # Wait for a valid GPS reference if available (improves NAV_TAKEOFF behavior in some SITL setups)
        if self.home_gps is None:
            return

        if not self.takeoff_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error("Takeoff service not available")
            self.log("/mavros/cmd/takeoff not available - entering RTL")
            self.transition_to(FlightState.RTL, "Takeoff service unavailable")
            return

        request = CommandTOL.Request()
        request.altitude = float(self.takeoff_altitude)
        request.latitude = float(self.home_gps[0])
        request.longitude = float(self.home_gps[1])
        request.min_pitch = 0.0
        request.yaw = float('nan')

        self.takeoff_client.call_async(request)
        self.get_logger().info(
            f"TAKEOFF CMD: requesting {self.takeoff_altitude}m at ({request.latitude:.6f},{request.longitude:.6f})"
        )
        self.log(
            f"MAVROS takeoff requested: alt={self.takeoff_altitude} lat={request.latitude:.6f} lon={request.longitude:.6f}"
        )

        self.has_taken_off_once = True
        self.transition_to(FlightState.WAIT_FOR_TAKEOFF, "Takeoff requested")

    def handle_wait_for_takeoff(self):
        """
        Monitor altitude climb until takeoff complete.

        Uses relative_altitude (barometer-based) as primary reference.
        Transition to NAVIGATE when ≥80% of target altitude reached.
        """
        if not self.fcu_state or not self.fcu_state.armed:
            self.get_logger().error("Disarmed during takeoff")
            self.log("Motors disarmed during climb - entering RTL")
            self.transition_to(FlightState.RTL, "Disarmed during takeoff")
            return

        # Check timeout
        if self.time_in_state() > self.takeoff_timeout:
            self.get_logger().error("Takeoff timeout - altitude not reached")
            self.log(f"Takeoff timeout: altitude {self.relative_altitude}m after {self.takeoff_timeout}s")
            self.transition_to(FlightState.RTL, "Takeoff timeout")
            return

        # DEBUG: Log current altitude every FSM cycle to diagnose stuck cases
        # Determine altitude source
        current_alt = self.relative_altitude
        alt_source = "relative_alt"
        
        # Fallback: if relative_alt is None after 2s, use local_pose.z
        if self.relative_altitude is None and self.time_in_state() > 2.0:
            if self.local_pose is not None:
                if not self.use_local_z_for_altitude:
                    self.get_logger().warn(
                        "MAVROS /mavros/global_position/relative_alt not available, "
                        "falling back to /mavros/local_position/pose.position.z"
                    )
                    self.use_local_z_for_altitude = True
                current_alt = self.local_pose.pose.position.z
                alt_source = "local_z"
        
        self.get_logger().info(
            f"[WAIT_FOR_TAKEOFF] alt={current_alt} ({alt_source}) target={self.takeoff_altitude * 0.8:.1f}m time={self.time_in_state():.1f}s"
        )

        # Monitor altitude
        if current_alt is None:
            self.get_logger().warn("Waiting for altitude data (relative_alt or local_pose.z)...")
            return

        target = self.takeoff_altitude * 0.8

        # Log progress every 2 seconds
        if int(self.time_in_state()) % 2 == 0 and self.time_in_state() > 0:
            self.get_logger().info(
                f"Climbing: {current_alt:.1f}m / {self.takeoff_altitude:.1f}m "
                f"(target: {target:.1f}m)"
            )

        if current_alt >= target:
            self.get_logger().info(f"Takeoff complete: {current_alt:.1f}m")
            self.log(f"Takeoff successful: altitude {current_alt:.1f}m reached")
            self.transition_to(FlightState.NAVIGATE, "Takeoff complete")

    def handle_navigate(self):
        """
        Navigate through waypoints using position control.

        Uses local ENU coordinates for precise position control.
        Detects arrival using XY distance in local frame.
        """
        if not self.fcu_state or not self.fcu_state.armed:
            self.get_logger().error("Disarmed during navigation")
            self.log("Motors disarmed during navigation - entering RTL")
            self.detection_pub.publish(Bool(data=False))
            self.transition_to(FlightState.RTL, "Disarmed during navigation")
            return

        if not self.waypoints_local or not self.local_pose:
            return

        # Build path if needed
        if not self.path_xy or len(self.path_xy) != len(self.waypoints_local):
            self._build_path_from_waypoints_local()
            self.path_progress_s = 0.0

        if len(self.path_xy) < 2:
            # Degenerate path: just hold at the single point
            x, y, _ = self.waypoints_local[0]
            self.current_target_local = (x, y, self.navigation_altitude)
            if self.distance_to_waypoint(self.current_target_local) < self.path_end_radius_m:
                self.transition_to(FlightState.MISSION_COMPLETE, "Single target reached")
                self.detection_pub.publish(Bool(data=False))
            return

        # Update lookahead target along the path
        target_xyz, remaining_m = self._compute_pursuit_target()
        self.current_target_local = target_xyz

        # Update and publish the "next waypoint" monitor topic based on path progress
        self._update_next_waypoint_from_progress()

        # Periodic debug (helps SITL tuning without spamming)
        sec = int(self.time_in_state())
        if sec != self._last_nav_log_sec and sec % 2 == 0:
            self._last_nav_log_sec = sec
            self.get_logger().info(
                f"NAV: s={self.path_progress_s:.1f}/{self.path_length_m:.1f}m "
                f"rem={remaining_m:.1f}m target=({target_xyz[0]:.1f},{target_xyz[1]:.1f},{target_xyz[2]:.1f})"
            )

        # Mission complete when close to path end
        end_x, end_y = self.path_xy[-1]
        end_dist = math.hypot(end_x - self.local_pose.pose.position.x, end_y - self.local_pose.pose.position.y)
        if end_dist <= self.path_end_radius_m and remaining_m <= self.path_end_radius_m:
            self.transition_to(FlightState.MISSION_COMPLETE, "Path end reached")
            self.detection_pub.publish(Bool(data=False))

    def handle_mission_complete(self):
        """Mission finished - initiate RTL."""
        self.get_logger().info("=" * 60)
        self.get_logger().info("MISSION COMPLETE - All waypoints reached")
        self.get_logger().info("=" * 60)
        self.log("Mission complete - all waypoints reached")
        self.transition_to(FlightState.RTL, "Mission complete")

    # ========================================================================
    # SETPOINT PUBLISHING
    # ========================================================================

    def publish_setpoint(self):
        """
        Publish position setpoints at 10Hz.

        CRITICAL: Must stream continuously from SET_GUIDED state onwards.
        ArduPilot GUIDED mode will auto-disarm without setpoints.

        Setpoint selection:
          SET_GUIDED/ARM/TAKEOFF_CMD/WAIT_FOR_TAKEOFF: Current position at target altitude
          NAVIGATE: Current waypoint position
          Others: No setpoints (not in active flight)
        """
        # Only publish during active flight states (including SET_GUIDED for early streaming)
        # CRITICAL: Exclude TAKEOFF_CMD and early WAIT_FOR_TAKEOFF to prevent position setpoint
        # conflict with MAVROS takeoff command
        if self.state not in [FlightState.SET_GUIDED, FlightState.ARM, FlightState.NAVIGATE]:
            return
        
        # For WAIT_FOR_TAKEOFF, only publish after drone starts climbing
        # This allows the MAVROS takeoff command to execute without interference
        if self.state == FlightState.WAIT_FOR_TAKEOFF:
            # Wait for either 5 seconds OR 5m altitude before resuming setpoints
            time_ok = self.time_in_state() >= 5.0
            alt_ok = False
            
            if self.relative_altitude is not None:
                alt_ok = self.relative_altitude >= 5.0
            elif self.use_local_z_for_altitude and self.local_pose is not None:
                alt_ok = self.local_pose.pose.position.z >= 5.0
            
            if not (time_ok or alt_ok):
                return  # Don't publish setpoints yet, let takeoff command work

        setpoint = PoseStamped()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.header.frame_id = 'map'

        if self.state in [FlightState.SET_GUIDED, FlightState.ARM, FlightState.WAIT_FOR_TAKEOFF]:
            # Pre-takeoff and during climb: hold current XY, target altitude Z
            if self.local_pose:
                setpoint.pose.position.x = self.local_pose.pose.position.x
                setpoint.pose.position.y = self.local_pose.pose.position.y
                setpoint.pose.position.z = self.takeoff_altitude
            else:
                # No valid local pose yet - skip setpoint to avoid confusing FCU with 0,0,alt
                return

        elif self.state == FlightState.NAVIGATE:
            # Navigation: target current pursuit point along path
            if self.current_target_local is not None:
                setpoint.pose.position.x = self.current_target_local[0]
                setpoint.pose.position.y = self.current_target_local[1]
                setpoint.pose.position.z = self.current_target_local[2]
            elif self.waypoints_local:
                # Fallback: first local waypoint
                setpoint.pose.position.x = self.waypoints_local[0][0]
                setpoint.pose.position.y = self.waypoints_local[0][1]
                setpoint.pose.position.z = self.navigation_altitude
            else:
                # Fallback: hover at current position
                setpoint.pose.position.x = self.local_pose.pose.position.x
                setpoint.pose.position.y = self.local_pose.pose.position.y
                setpoint.pose.position.z = self.navigation_altitude

        # Orientation: maintain current yaw or use identity
        if self.local_pose:
            setpoint.pose.orientation = self.local_pose.pose.orientation
        else:
            # Identity quaternion (no rotation)
            setpoint.pose.orientation.w = 1.0
            setpoint.pose.orientation.x = 0.0
            setpoint.pose.orientation.y = 0.0
            setpoint.pose.orientation.z = 0.0

        self.setpoint_pub.publish(setpoint)

    def publish_status(self):
        """Publish current state for monitoring."""
        status = String()
        status.data = self.state.name
        self.status_pub.publish(status)

    def publish_next_waypoint(self):
        """Publish current target waypoint in GPS coordinates."""
        if self.current_waypoint_idx >= len(self.waypoints_gps):
            return

        wp = self.waypoints_gps[self.current_waypoint_idx]
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.latitude = wp[0]
        msg.longitude = wp[1]
        msg.altitude = wp[2]

        self.waypoint_pub.publish(msg)

    # ========================================================================
    # COORDINATE CONVERSION
    # ========================================================================

    def convert_waypoints_to_local(self):
        """
        Convert GPS waypoints to local ENU coordinates.

        Uses home GPS position and current local pose as reference.
        This is a flat-earth approximation suitable for missions < 1km.
        """
        if not self.home_gps or not self.home_local_pose:
            self.get_logger().warn("Cannot convert waypoints - no home position")
            # Use simple offset method as fallback
            self._convert_waypoints_simple()
            return

        self.waypoints_local.clear()

        home_lat, home_lon = self.home_gps
        home_x = self.home_local_pose.pose.position.x
        home_y = self.home_local_pose.pose.position.y

        # Earth radius for flat-earth approximation
        R = 6371000.0  # meters

        for gps_wp in self.waypoints_gps:
            lat, lon, alt = gps_wp

            # Convert lat/lon delta to meters (flat-earth)
            dlat = math.radians(lat - home_lat)
            dlon = math.radians(lon - home_lon)

            # ENU: East = dlon * R * cos(lat), North = dlat * R
            east = dlon * R * math.cos(math.radians(home_lat))
            north = dlat * R

            # Add home offset
            x = home_x + east
            y = home_y + north
            z = self.navigation_altitude

            self.waypoints_local.append((x, y, z))

        self.get_logger().info(f"Converted {len(self.waypoints_local)} waypoints to local ENU")
        self.log(f"Waypoint conversion complete: {len(self.waypoints_local)} local targets")

        self._build_path_from_waypoints_local()

        # Publish first waypoint
        if self.waypoints_gps:
            self.publish_next_waypoint()

    def _convert_waypoints_simple(self):
        """
        Simple waypoint conversion using home_gps and current local pose.
        
        Uses flat-earth approximation from home GPS to convert waypoints
        to local ENU coordinates relative to current position.
        """
        if not self.local_pose or not self.home_gps:
            self.get_logger().error("Cannot convert waypoints - missing local_pose or home_gps")
            return

        self.waypoints_local.clear()

        home_lat, home_lon = self.home_gps
        
        # Use current position as reference origin
        ref_x = self.local_pose.pose.position.x
        ref_y = self.local_pose.pose.position.y

        # Earth radius for flat-earth approximation
        R = 6371000.0  # meters

        for gps_wp in self.waypoints_gps:
            lat, lon, alt = gps_wp

            # Convert lat/lon delta from home to meters (flat-earth)
            dlat = math.radians(lat - home_lat)
            dlon = math.radians(lon - home_lon)

            # ENU: East = dlon * R * cos(lat), North = dlat * R
            east = dlon * R * math.cos(math.radians(home_lat))
            north = dlat * R

            # Convert to local frame (current position is at ref_x, ref_y)
            x = ref_x + east
            y = ref_y + north
            z = self.navigation_altitude

            self.waypoints_local.append((x, y, z))

        self.get_logger().info(f"Converted {len(self.waypoints_local)} waypoints to local ENU (simple)")
        self.log(f"Waypoint conversion (simple): {len(self.waypoints_local)} local targets")

        self._build_path_from_waypoints_local()

    # ========================================================================
    # PATH FOLLOWING (PURE PURSUIT STYLE)
    # ========================================================================

    def _build_path_from_waypoints_local(self):
        """Build a polyline path from local waypoints."""
        self.path_xy = [(wp[0], wp[1]) for wp in self.waypoints_local]
        self.path_cum_s = [0.0]
        total = 0.0
        for i in range(len(self.path_xy) - 1):
            x0, y0 = self.path_xy[i]
            x1, y1 = self.path_xy[i + 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            total += seg
            self.path_cum_s.append(total)
        self.path_length_m = total

    def _compute_pursuit_target(self) -> Tuple[Tuple[float, float, float], float]:
        """Compute a lookahead target along the path and remaining distance to end."""
        assert self.local_pose is not None
        x = self.local_pose.pose.position.x
        y = self.local_pose.pose.position.y

        closest_s = self._closest_s_on_path(x, y)

        # Monotonic progress (prevents noisy backtracking)
        if closest_s > self.path_progress_s:
            self.path_progress_s = closest_s

        target_s = min(self.path_progress_s + float(self.path_lookahead_m), self.path_length_m)
        tx, ty = self._point_at_s(target_s)

        # Clamp how far the target can be from current position (safety)
        dx = tx - x
        dy = ty - y
        dist = math.hypot(dx, dy)
        max_step = float(self.path_max_target_step_m)
        if max_step > 0.0 and dist > max_step:
            scale = max_step / dist
            tx = x + dx * scale
            ty = y + dy * scale

        remaining = max(0.0, self.path_length_m - self.path_progress_s)
        return (tx, ty, self.navigation_altitude), remaining

    def _closest_s_on_path(self, x: float, y: float) -> float:
        """Return arc-length s of the closest point on the polyline path."""
        best_s = 0.0
        best_d2 = float('inf')

        for i in range(len(self.path_xy) - 1):
            x0, y0 = self.path_xy[i]
            x1, y1 = self.path_xy[i + 1]
            vx = x1 - x0
            vy = y1 - y0
            seg_len2 = vx * vx + vy * vy
            if seg_len2 <= 1e-9:
                # Degenerate segment
                dx = x - x0
                dy = y - y0
                d2 = dx * dx + dy * dy
                if d2 < best_d2:
                    best_d2 = d2
                    best_s = self.path_cum_s[i]
                continue

            wx = x - x0
            wy = y - y0
            t = (wx * vx + wy * vy) / seg_len2
            if t < 0.0:
                t = 0.0
            elif t > 1.0:
                t = 1.0

            cx = x0 + t * vx
            cy = y0 + t * vy
            dx = x - cx
            dy = y - cy
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                seg_len = math.sqrt(seg_len2)
                best_s = self.path_cum_s[i] + t * seg_len

        return best_s

    def _point_at_s(self, s: float) -> Tuple[float, float]:
        """Interpolate a point on the polyline at arc-length s."""
        if s <= 0.0:
            return self.path_xy[0]
        if s >= self.path_length_m:
            return self.path_xy[-1]

        for i in range(len(self.path_xy) - 1):
            s0 = self.path_cum_s[i]
            s1 = self.path_cum_s[i + 1]
            if s <= s1:
                x0, y0 = self.path_xy[i]
                x1, y1 = self.path_xy[i + 1]
                seg = s1 - s0
                if seg <= 1e-9:
                    return x1, y1
                t = (s - s0) / seg
                return (x0 + t * (x1 - x0), y0 + t * (y1 - y0))

        return self.path_xy[-1]

    def _update_next_waypoint_from_progress(self):
        """Advance `current_waypoint_idx` based on path progress and publish `/drone1/next_waypoint`."""
        if not self.path_cum_s or not self.waypoints_gps:
            return

        # Find the first waypoint whose arc-length is ahead of our progress.
        # Small epsilon avoids flapping around exact waypoint boundaries.
        eps = 0.5
        next_idx = len(self.path_cum_s) - 1
        for i, s_i in enumerate(self.path_cum_s):
            if s_i > self.path_progress_s + eps:
                next_idx = i
                break

        # Clamp to available GPS list (should match length, but be safe)
        if next_idx >= len(self.waypoints_gps):
            next_idx = len(self.waypoints_gps) - 1

        if next_idx < 0:
            return

        self.current_waypoint_idx = next_idx

        if self.current_waypoint_idx != self._last_published_waypoint_idx:
            self._last_published_waypoint_idx = self.current_waypoint_idx
            self.publish_next_waypoint()

    def distance_to_waypoint(self, target: Tuple[float, float, float]) -> float:
        """Calculate XY distance to target in local frame (meters)."""
        if not self.local_pose:
            return float('inf')

        dx = target[0] - self.local_pose.pose.position.x
        dy = target[1] - self.local_pose.pose.position.y

        return math.sqrt(dx * dx + dy * dy)

    # ========================================================================
    # FAILSAFE
    # ========================================================================

    def request_rtl(self):
        """Request RTL mode as failsafe."""
        if not self.fcu_state:
            return

        if self.fcu_state.mode == 'RTL':
            return

        if self.mode_client.wait_for_service(timeout_sec=0.5):
            request = SetMode.Request()
            request.custom_mode = 'RTL'
            future = self.mode_client.call_async(request)
            self.get_logger().info("RTL mode requested")
            self.log("RTL mode requested")

    def destroy_node(self):
        """Cleanup on shutdown."""
        self.log("Node shutting down")
        self.log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = Drone1FlightController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
