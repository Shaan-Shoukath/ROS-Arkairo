#!/usr/bin/env python3
# Copyright 2024 Shaan Shoukath
# SPDX-License-Identifier: Apache-2.0

"""
Drone-2 Autonomous Navigation Controller
=========================================
Hardware: Cube Orange+ | Companion: Raspberry Pi 5
ROS: Jazzy | Middleware: MAVROS | Mode: GUIDED ONLY

Unified flight controller for Drone-2 sprayer missions.
Modeled after drone1_navigation for consistent behavior.

State Machine:
    IDLE → WAIT_FCU → WAIT_TARGET → SET_GUIDED → ARM →
    TAKEOFF → WAIT_TAKEOFF → NAVIGATE → ARRIVED →
    WAIT_FOR_NEXT → [NAVIGATE or RTL] → LANDED

Critical Design Rules (same as Drone-1):
  ✅ MAV_CMD_NAV_TAKEOFF is the ONLY way to initiate flight
  ✅ Position setpoints stream at 10Hz starting from ARM state
  ✅ ARM happens exactly once
  ✅ TAKEOFF happens exactly once
  ✅ Throttle/PWM controlled exclusively by ArduPilot
  ✅ No RC override, no velocity-based takeoff, no AUTO mode

Subscribers:
    /drone2/target_position (sensor_msgs/NavSatFix): Target from telem_rx
    /mavros/state (mavros_msgs/State): FCU connection & mode
    /mavros/global_position/global (sensor_msgs/NavSatFix): Current GPS
    /mavros/global_position/relative_alt (std_msgs/Float64): Barometer altitude
    /mavros/local_position/pose (geometry_msgs/PoseStamped): Local position

Publishers:
    /mavros/setpoint_position/local (geometry_msgs/PoseStamped): Setpoints
    /drone2/arrival_status (std_msgs/Bool): Arrival at target
    /drone2/navigation_status (std_msgs/String): State for monitoring
"""

import math
import os
from datetime import datetime
from enum import Enum, auto
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, String, Float64
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, ParamSetV2
from rcl_interfaces.msg import ParameterValue, ParameterType


class FlightState(Enum):
    """Flight state machine."""
    IDLE = auto()
    WAIT_FCU = auto()
    WAIT_TARGET = auto()
    SET_GUIDED = auto()
    ARM = auto()
    TAKEOFF = auto()
    WAIT_TAKEOFF = auto()
    NAVIGATE = auto()
    ARRIVED = auto()
    WAIT_SPRAY = auto()     # Wait for centering + spray to complete
    WAIT_FOR_NEXT = auto()
    RTL = auto()
    LANDED = auto()


class Drone2NavigationNode(Node):
    """
    Unified autonomous flight controller for Drone-2.
    
    Designed for real hardware (Cube Orange+ / Raspberry Pi 5) and SITL testing.
    Uses GUIDED mode exclusively with position-based control.
    """
    
    EARTH_RADIUS = 6371000  # meters
    
    def __init__(self):
        super().__init__('drone2_navigation_node')
        
        # ================================================================
        # PARAMETERS (matching Drone-1 structure)
        # ================================================================
        self.declare_parameter('takeoff_altitude_m', 10.0)
        self.declare_parameter('navigation_altitude_m', 10.0)  # Guided height for navigation
        self.declare_parameter('arrival_radius_m', 3.0)
        self.declare_parameter('setpoint_rate_hz', 10.0)
        self.declare_parameter('wait_timeout_sec', 10.0)  # Wait 10s for next geotag before RTL
        self.declare_parameter('takeoff_timeout_sec', 60.0)
        self.declare_parameter('fcu_timeout_sec', 30.0)
        
        self.takeoff_alt = self.get_parameter('takeoff_altitude_m').value
        self.navigation_alt = self.get_parameter('navigation_altitude_m').value
        self.arrival_radius = self.get_parameter('arrival_radius_m').value
        self.setpoint_rate = self.get_parameter('setpoint_rate_hz').value
        self.wait_timeout = self.get_parameter('wait_timeout_sec').value
        self.takeoff_timeout = self.get_parameter('takeoff_timeout_sec').value
        self.fcu_timeout = self.get_parameter('fcu_timeout_sec').value
        
        # ================================================================
        # STATE VARIABLES
        # ================================================================
        self.state = FlightState.IDLE
        self.state_entry_time = self.get_clock().now()
        
        # FCU
        self.fcu_connected = False
        self.fcu_state: Optional[State] = None
        self.has_armed_once = False
        self.has_taken_off_once = False
        
        # Position tracking
        self.current_gps: Optional[NavSatFix] = None
        self.local_pose: Optional[PoseStamped] = None
        self.relative_altitude: Optional[float] = None  # Barometer altitude (like Drone-1)
        self.use_local_z_for_altitude = False  # Fallback if relative_alt unavailable
        self.home_gps: Optional[Tuple[float, float]] = None
        self.home_local: Optional[Tuple[float, float, float]] = None
        
        # Target
        self.target_gps: Optional[NavSatFix] = None
        self.target_local: Optional[Tuple[float, float, float]] = None
        self.targets_completed = 0
        
        # RTL tracking - can resume from timeout RTL but not emergency RTL
        self.rtl_due_to_timeout = False
        
        # SITL configuration flag
        self.params_configured = False
        
        # ================================================================
        # QOS
        # ================================================================
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # ================================================================
        # SUBSCRIBERS
        # ================================================================
        # QoS for target_position: BEST_EFFORT reliability (matches ros2 topic pub)
        # TRANSIENT_LOCAL durability allows receiving from both VOLATILE and TRANSIENT_LOCAL publishers
        target_pos_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.create_subscription(
            NavSatFix, '/drone2/target_position',
            self.target_callback, target_pos_qos
        )
        self.create_subscription(
            State, '/mavros/state',
            self.state_callback, sensor_qos
        )
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global',
            self.gps_callback, sensor_qos
        )
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.pose_callback, sensor_qos
        )
        # Relative altitude for accurate takeoff detection (like Drone-1)
        self.create_subscription(
            Float64, '/mavros/global_position/relative_alt',
            self.relative_alt_callback, sensor_qos
        )
        
        # Spray completion signal from sprayer node
        self.create_subscription(
            Bool, '/drone2/spray_done',
            self.spray_done_callback, reliable_qos
        )
        
        # ================================================================
        # PUBLISHERS
        # ================================================================
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10
        )
        self.arrival_pub = self.create_publisher(
            Bool, '/drone2/arrival_status', reliable_qos
        )
        self.status_pub = self.create_publisher(
            String, '/drone2/navigation_status', 10
        )
        
        # ================================================================
        # SERVICE CLIENTS
        # ================================================================
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.param_client = self.create_client(ParamSetV2, '/mavros/param/set')
        
        # ================================================================
        # TIMERS
        # ================================================================
        self.create_timer(0.5, self.fsm_update)
        self.create_timer(1.0 / self.setpoint_rate, self.publish_setpoint)
        self.create_timer(1.0, self.publish_status)
        
        # ================================================================
        # LOGGING (like Drone-1)
        # ================================================================
        log_dir = os.path.expanduser('~/Documents/ROSArkairo/logs')
        os.makedirs(log_dir, exist_ok=True)
        log_file = os.path.join(log_dir, 'drone2_flight_latest.log')
        self.log_file = open(log_file, 'w')
        self.log(f"Drone-2 Flight Controller initialized at {datetime.now()}")
        self.log(f"Takeoff altitude: {self.takeoff_alt}m")
        self.log(f"Navigation altitude: {self.navigation_alt}m")
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Drone-2 Autonomous Navigation Controller - Position Control Mode')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Takeoff: {self.takeoff_alt}m | Navigate: {self.navigation_alt}m')
        self.get_logger().info(f'Arrival radius: {self.arrival_radius}m')
        self.get_logger().info(f'Wait timeout: {self.wait_timeout}s')
        self.get_logger().info('Waiting for target from telem_rx...')
    
    def log(self, message: str):
        """Write to both ROS logger and file."""
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        self.log_file.write(f"[{timestamp}] {message}\n")
        self.log_file.flush()
    
    # ====================================================================
    # STATE TRANSITIONS
    # ====================================================================
    
    def transition_to(self, new_state: FlightState, reason: str = ""):
        """Transition to new state."""
        old = self.state
        self.state = new_state
        self.state_entry_time = self.get_clock().now()
        msg = f'State: {old.name} → {new_state.name}'
        if reason:
            msg += f' ({reason})'
        self.get_logger().info(msg)
    
    def time_in_state(self) -> float:
        """Seconds in current state."""
        return (self.get_clock().now() - self.state_entry_time).nanoseconds / 1e9
    
    # ====================================================================
    # CALLBACKS
    # ====================================================================
    
    def target_callback(self, msg: NavSatFix):
        """Receive target from telem_rx."""
        self.target_gps = msg
        self.get_logger().info(
            f'Target received: lat={msg.latitude:.6f}, '
            f'lon={msg.longitude:.6f}, alt={msg.altitude:.1f}m'
        )
        
        # Convert to local if we have home - use navigation altitude (guided height)
        if self.home_gps and self.home_local:
            self.target_local = self.gps_to_local(
                msg.latitude, msg.longitude, self.navigation_alt
            )
            self.get_logger().info(
                f'Target local: x={self.target_local[0]:.1f}, '
                f'y={self.target_local[1]:.1f}, z={self.target_local[2]:.1f}'
            )
        
        # Trigger flight or update target based on current state
        if self.state == FlightState.WAIT_TARGET:
            self.transition_to(FlightState.SET_GUIDED, 'First target received')
        elif self.state == FlightState.WAIT_FOR_NEXT:
            self.transition_to(FlightState.NAVIGATE, 'New target during wait')
        elif self.state in [FlightState.NAVIGATE, FlightState.ARRIVED]:
            # New target while flying - just update target and continue navigation
            self.get_logger().info('New target received during flight - redirecting!')
            self.log(f'Redirecting to new target: ({msg.latitude:.6f}, {msg.longitude:.6f})')
            # Stay in NAVIGATE (or move to NAVIGATE if in ARRIVED)
            if self.state == FlightState.ARRIVED:
                self.transition_to(FlightState.NAVIGATE, 'New target during arrival')
        elif self.state == FlightState.RTL and self.rtl_due_to_timeout:
            # Resume from timeout-RTL if new geotag received
            # Drone is already in the air - go directly to NAVIGATE (skip ARM/TAKEOFF)
            self.get_logger().info('New geotag received during RTL - resuming navigation!')
            self.log('Resuming from timeout-RTL due to new geotag')
            self.rtl_due_to_timeout = False
            # Go directly to NAVIGATE since we're already airborne
            self.transition_to(FlightState.NAVIGATE, 'Resume from RTL - new geotag')
        elif self.state == FlightState.LANDED:
            # New geotag after landing - restart full flight sequence
            self.get_logger().info('New geotag received after landing - restarting flight!')
            self.log(f'Restarting for new target: ({msg.latitude:.6f}, {msg.longitude:.6f})')
            self.transition_to(FlightState.SET_GUIDED, 'New target after landing')
    
    def relative_alt_callback(self, msg: Float64):
        """Track relative altitude for takeoff detection (barometer-based)."""
        self.relative_altitude = msg.data
    
    def spray_done_callback(self, msg: Bool):
        """Handle spray completion signal from sprayer node."""
        if msg.data and self.state == FlightState.WAIT_SPRAY:
            self.get_logger().info('Spray complete - ready for next target')
            self.log('Spray operation completed')
            self.transition_to(FlightState.WAIT_FOR_NEXT, 'Spray complete')
    
    def state_callback(self, msg: State):
        """Monitor FCU connection and mode."""
        self.fcu_state = msg
        
        # Detect connection
        if msg.connected and not self.fcu_connected:
            self.fcu_connected = True
            self.get_logger().info('FCU connected')
            self.log('FCU connection established')
        
        # Detect disconnection - emergency RTL
        if not msg.connected and self.fcu_connected:
            self.get_logger().error('FCU DISCONNECTED - Emergency RTL')
            self.log('FCU connection lost - initiating RTL')
            self.transition_to(FlightState.RTL, 'FCU disconnected')
        
        # Detect mode exit during flight - emergency RTL
        if self.state in [FlightState.ARM, FlightState.TAKEOFF,
                          FlightState.WAIT_TAKEOFF, FlightState.NAVIGATE]:
            if msg.mode != 'GUIDED':
                self.get_logger().error(f'Lost GUIDED mode (now {msg.mode}) - Emergency RTL')
                self.log(f'Mode changed from GUIDED to {msg.mode} - initiating RTL')
                self.transition_to(FlightState.RTL, 'Lost GUIDED mode')
    
    def gps_callback(self, msg: NavSatFix):
        """Update current GPS and capture home."""
        self.current_gps = msg
        if self.home_gps is None and msg.latitude != 0.0:
            self.home_gps = (msg.latitude, msg.longitude)
            self.get_logger().info(
                f'Home GPS: ({msg.latitude:.6f}, {msg.longitude:.6f})'
            )
    
    def pose_callback(self, msg: PoseStamped):
        """Update local position and capture home."""
        self.local_pose = msg
        if self.home_local is None and self.home_gps is not None:
            self.home_local = (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            )
            self.get_logger().info(
                f'Home local: ({self.home_local[0]:.1f}, '
                f'{self.home_local[1]:.1f}, {self.home_local[2]:.1f})'
            )
            
            # Convert target if pending
            if self.target_gps and self.target_local is None:
                self.target_local = self.gps_to_local(
                    self.target_gps.latitude,
                    self.target_gps.longitude,
                    self.takeoff_alt
                )
    
    # ====================================================================
    # GPS TO LOCAL CONVERSION
    # ====================================================================
    
    def gps_to_local(self, lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
        """Convert GPS to local ENU coordinates."""
        if self.home_gps is None or self.home_local is None:
            return (0.0, 0.0, alt)
        
        home_lat, home_lon = self.home_gps
        
        # Approximate conversion (good for small distances)
        lat_rad = math.radians(home_lat)
        
        dlat = lat - home_lat
        dlon = lon - home_lon
        
        # Meters per degree
        m_per_deg_lat = 111132.92
        m_per_deg_lon = 111132.92 * math.cos(lat_rad)
        
        # ENU: East = X, North = Y
        x = dlon * m_per_deg_lon + self.home_local[0]
        y = dlat * m_per_deg_lat + self.home_local[1]
        z = alt
        
        return (x, y, z)
    
    # ====================================================================
    # FSM UPDATE
    # ====================================================================
    
    def fsm_update(self):
        """Main state machine (2Hz)."""
        
        if self.state == FlightState.IDLE:
            self.transition_to(FlightState.WAIT_FCU, 'Starting')
        
        elif self.state == FlightState.WAIT_FCU:
            if self.fcu_connected:
                self.transition_to(FlightState.WAIT_TARGET, 'FCU ready')
            elif self.time_in_state() > self.fcu_timeout:
                self.get_logger().error('FCU timeout')
        
        elif self.state == FlightState.WAIT_TARGET:
            # Waiting for target_callback to trigger transition
            pass
        
        elif self.state == FlightState.SET_GUIDED:
            self.handle_set_guided()
        
        elif self.state == FlightState.ARM:
            self.handle_arm()
        
        elif self.state == FlightState.TAKEOFF:
            self.handle_takeoff()
        
        elif self.state == FlightState.WAIT_TAKEOFF:
            self.handle_wait_takeoff()
        
        elif self.state == FlightState.NAVIGATE:
            self.handle_navigate()
        
        elif self.state == FlightState.ARRIVED:
            self.handle_arrived()
        
        elif self.state == FlightState.WAIT_SPRAY:
            self.handle_wait_spray()
        
        elif self.state == FlightState.WAIT_FOR_NEXT:
            self.handle_wait_for_next()
        
        elif self.state == FlightState.RTL:
            self.handle_rtl()
    
    # ====================================================================
    # STATE HANDLERS
    # ====================================================================
    
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
            
            self.transition_to(FlightState.ARM, 'GUIDED mode confirmed')
            self.log('GUIDED mode confirmed - proceeding to ARM')
            return
        
        # Request GUIDED mode (rate limited)
        if self.time_in_state() < 1.0:
            return
        
        if self.mode_client.wait_for_service(timeout_sec=0.5):
            req = SetMode.Request()
            req.custom_mode = 'GUIDED'
            self.mode_client.call_async(req)
            self.get_logger().info('Requesting GUIDED mode')
            self.log('GUIDED mode request sent')
    
    def configure_sitl_params(self):
        """Set ArduPilot parameters for SITL testing (disable auto-disarm)."""
        if not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Param service not available - skipping SITL config')
            return
        
        # Set DISARM_DELAY to 0 (disable auto-disarm on ground)
        request = ParamSetV2.Request()
        request.param_id = 'DISARM_DELAY'
        request.value = ParameterValue()
        request.value.type = ParameterType.PARAMETER_INTEGER
        request.value.integer_value = 0
        
        self.param_client.call_async(request)
        self.get_logger().info('Setting DISARM_DELAY=0 for SITL testing')
        self.log('DISARM_DELAY=0 set via MAVROS')
    
    def handle_arm(self):
        """Stream setpoints then arm."""
        if not self.fcu_state:
            return
        
        if self.fcu_state.mode != 'GUIDED':
            self.transition_to(FlightState.SET_GUIDED, 'Lost GUIDED')
            return
        
        if self.fcu_state.armed:
            self.has_armed_once = True
            self.transition_to(FlightState.TAKEOFF, 'Armed')
            return
        
        # Stream setpoints for 3 seconds before arming
        if self.time_in_state() < 3.0:
            if int(self.time_in_state() * 2) % 2 == 0:
                self.get_logger().info(
                    f'Streaming setpoints... ({self.time_in_state():.1f}s / 3.0s)'
                )
            return
        
        # Send arm command every 2 seconds
        time_since_3s = self.time_in_state() - 3.0
        if int(time_since_3s) % 2 == 0 and (time_since_3s % 2.0) < 0.6:
            if self.arming_client.wait_for_service(timeout_sec=0.5):
                req = CommandBool.Request()
                req.value = True
                self.arming_client.call_async(req)
                self.get_logger().info('ARM command sent')
        
        if self.time_in_state() > 18.0:
            self.get_logger().error('ARM timeout')
            self.transition_to(FlightState.RTL, 'ARM failed')
    
    def handle_takeoff(self):
        """Send takeoff command."""
        if not self.fcu_state or not self.fcu_state.armed:
            self.get_logger().error('Disarmed before takeoff')
            return
        
        if self.has_taken_off_once:
            return
        
        if self.time_in_state() < 1.0:
            return
        
        if self.home_gps is None:
            return
        
        if self.takeoff_client.wait_for_service(timeout_sec=0.5):
            req = CommandTOL.Request()
            req.altitude = float(self.takeoff_alt)
            req.latitude = float(self.home_gps[0])
            req.longitude = float(self.home_gps[1])
            req.min_pitch = 0.0
            req.yaw = float('nan')
            self.takeoff_client.call_async(req)
            self.get_logger().info(f'TAKEOFF to {self.takeoff_alt}m')
            self.has_taken_off_once = True
            self.transition_to(FlightState.WAIT_TAKEOFF, 'Takeoff requested')
    
    def handle_wait_takeoff(self):
        """
        Monitor altitude climb until takeoff complete.
        
        Uses relative_altitude (barometer-based) as primary reference.
        Transition to NAVIGATE when ≥80% of target altitude reached.
        """
        if not self.fcu_state or not self.fcu_state.armed:
            self.get_logger().error('Disarmed during takeoff')
            self.log('Motors disarmed during climb - entering RTL')
            self.transition_to(FlightState.RTL, 'Disarmed during takeoff')
            return
        
        # Check timeout
        if self.time_in_state() > self.takeoff_timeout:
            self.get_logger().error('Takeoff timeout - altitude not reached')
            self.log(f'Takeoff timeout: altitude {self.relative_altitude}m after {self.takeoff_timeout}s')
            self.transition_to(FlightState.RTL, 'Takeoff timeout')
            return
        
        # Determine altitude source (like Drone-1)
        current_alt = self.relative_altitude
        alt_source = 'relative_alt'
        
        # Fallback: if relative_alt is None after 2s, use local_pose.z
        if self.relative_altitude is None and self.time_in_state() > 2.0:
            if self.local_pose is not None:
                if not self.use_local_z_for_altitude:
                    self.get_logger().warn(
                        'MAVROS /mavros/global_position/relative_alt not available, '
                        'falling back to /mavros/local_position/pose.position.z'
                    )
                    self.use_local_z_for_altitude = True
                current_alt = self.local_pose.pose.position.z
                alt_source = 'local_z'
        
        self.get_logger().info(
            f'[WAIT_TAKEOFF] alt={current_alt} ({alt_source}) target={self.takeoff_alt * 0.8:.1f}m time={self.time_in_state():.1f}s'
        )
        
        # Monitor altitude
        if current_alt is None:
            self.get_logger().warn('Waiting for altitude data (relative_alt or local_pose.z)...')
            return
        
        target = self.takeoff_alt * 0.8
        
        # Log progress every 2 seconds
        if int(self.time_in_state()) % 2 == 0 and self.time_in_state() > 0:
            self.get_logger().info(
                f'Climbing: {current_alt:.1f}m / {self.takeoff_alt:.1f}m '
                f'(target: {target:.1f}m)'
            )
        
        if current_alt >= target:
            self.get_logger().info(f'Takeoff complete: {current_alt:.1f}m')
            self.log(f'Takeoff successful: altitude {current_alt:.1f}m reached')
            self.transition_to(FlightState.NAVIGATE, 'Takeoff complete')
    
    def handle_navigate(self):
        """Navigate to target."""
        if not self.fcu_state or not self.fcu_state.armed:
            self.get_logger().error('Disarmed during navigation')
            self.transition_to(FlightState.RTL, 'Disarmed')
            return
        
        # Ensure we're in GUIDED mode (needed when resuming from RTL)
        if self.fcu_state.mode != 'GUIDED':
            # Request GUIDED mode (rate limited)
            if self.time_in_state() < 10 and int(self.time_in_state() * 2) % 2 == 0:
                if self.mode_client.wait_for_service(timeout_sec=0.1):
                    req = SetMode.Request()
                    req.custom_mode = 'GUIDED'
                    self.mode_client.call_async(req)
                    self.get_logger().info(f'Requesting GUIDED mode (currently: {self.fcu_state.mode})')
            return  # Wait for GUIDED mode
        
        if self.target_local is None or self.local_pose is None:
            return
        
        # Calculate distance
        dx = self.target_local[0] - self.local_pose.pose.position.x
        dy = self.target_local[1] - self.local_pose.pose.position.y
        dist = math.sqrt(dx*dx + dy*dy)
        
        if int(self.time_in_state()) % 3 == 0:
            self.get_logger().info(f'Distance to target: {dist:.1f}m')
        
        if dist < self.arrival_radius:
            self.transition_to(FlightState.ARRIVED, 'At target')
    
    def handle_arrived(self):
        """Handle arrival at target - notify centering node and wait for spray."""
        self.targets_completed += 1
        self.get_logger().info(f'Arrived! Targets completed: {self.targets_completed}')
        
        # Publish arrival → triggers Detection/Centering → then Sprayer
        msg = Bool()
        msg.data = True
        self.arrival_pub.publish(msg)
        
        # Wait for spray_done signal before looking for next target
        self.transition_to(FlightState.WAIT_SPRAY, 'Waiting for centering + spray')
    
    def handle_wait_spray(self):
        """Wait for centering and spray to complete."""
        # Log progress every 5 seconds
        if int(self.time_in_state()) % 5 == 0 and self.time_in_state() > 0:
            self.get_logger().info(
                f'[WAIT_SPRAY] Waiting for centering + spray... {int(self.time_in_state())}s'
            )
        
        # Timeout after 60 seconds (centering ~30s + spray ~15s + buffer)
        if self.time_in_state() > 60.0:
            self.get_logger().warn('Spray timeout - continuing to next target')
            self.log('Spray operation timed out after 60s')
            self.transition_to(FlightState.WAIT_FOR_NEXT, 'Spray timeout')
    
    def handle_wait_for_next(self):
        """Wait at current position for next target, then RTL if timeout."""
        time_in = self.time_in_state()
        time_remaining = max(0, self.wait_timeout - time_in)
        
        # Log countdown every second
        elapsed_seconds = int(time_in)
        if elapsed_seconds != getattr(self, '_last_wait_log_second', -1):
            self._last_wait_log_second = elapsed_seconds
            self.get_logger().info(
                f'[WAIT_FOR_NEXT] Hovering at waypoint... '
                f'{int(time_remaining)}s until RTL'
            )
        
        # Timeout reached - initiate RTL
        if time_in >= self.wait_timeout:
            self.get_logger().info(f'{self.wait_timeout}s timeout reached - initiating RTL')
            self.rtl_due_to_timeout = True  # Mark as resumable
            self._last_wait_log_second = -1  # Reset for next wait period
            self.transition_to(FlightState.RTL, 'Wait timeout')
    
    def handle_rtl(self):
        """Request RTL mode and monitor landing."""
        if not self.fcu_state:
            return
        
        # Request RTL mode if not already in it
        if self.fcu_state.mode != 'RTL':
            # Request every 2 seconds until confirmed
            if int(self.time_in_state()) % 2 == 0:
                if self.mode_client.wait_for_service(timeout_sec=0.5):
                    req = SetMode.Request()
                    req.custom_mode = 'RTL'
                    self.mode_client.call_async(req)
                    self.get_logger().info(f'RTL mode requested (current: {self.fcu_state.mode})')
        else:
            # Log RTL progress every 3 seconds
            if int(self.time_in_state()) % 3 == 0:
                alt = self.relative_altitude if self.relative_altitude else 0.0
                self.get_logger().info(f'[RTL] Returning home... altitude: {alt:.1f}m')
        
        # Check if landed (disarmed while in RTL)
        if self.fcu_state.mode == 'RTL' and not self.fcu_state.armed:
            self.get_logger().info('RTL complete - landed and disarmed')
            self.transition_to(FlightState.LANDED, 'RTL complete')
    
    # ====================================================================
    # SETPOINT PUBLISHING (10Hz)
    # ====================================================================
    
    def publish_setpoint(self):
        """Publish position setpoints continuously."""
        # Only during active flight states
        if self.state not in [
            FlightState.SET_GUIDED, FlightState.ARM,
            FlightState.NAVIGATE, FlightState.ARRIVED,
            FlightState.WAIT_SPRAY, FlightState.WAIT_FOR_NEXT
        ]:
            return
        
        # Skip early takeoff to let MAVROS takeoff work
        if self.state == FlightState.WAIT_TAKEOFF:
            if self.time_in_state() < 5.0:
                return
        
        if self.local_pose is None:
            return
        
        setpoint = PoseStamped()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.header.frame_id = 'map'
        
        if self.state in [FlightState.SET_GUIDED, FlightState.ARM]:
            # Hold position, target altitude
            setpoint.pose.position.x = self.local_pose.pose.position.x
            setpoint.pose.position.y = self.local_pose.pose.position.y
            setpoint.pose.position.z = self.takeoff_alt
        
        elif self.state in [FlightState.NAVIGATE, FlightState.ARRIVED, 
                           FlightState.WAIT_SPRAY, FlightState.WAIT_FOR_NEXT]:
            # Navigate to target
            if self.target_local:
                setpoint.pose.position.x = self.target_local[0]
                setpoint.pose.position.y = self.target_local[1]
                setpoint.pose.position.z = self.target_local[2]
            else:
                setpoint.pose.position.x = self.local_pose.pose.position.x
                setpoint.pose.position.y = self.local_pose.pose.position.y
                setpoint.pose.position.z = self.navigation_alt
        
        # Keep current orientation
        setpoint.pose.orientation = self.local_pose.pose.orientation
        
        self.setpoint_pub.publish(setpoint)
    
    def publish_status(self):
        """Publish current state."""
        msg = String()
        msg.data = self.state.name
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Drone2NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'Targets completed: {node.targets_completed}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
