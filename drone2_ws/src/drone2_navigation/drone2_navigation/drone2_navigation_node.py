#!/usr/bin/env python3
# Copyright 2024 Maintainer
# SPDX-License-Identifier: Apache-2.0

"""
Drone-2 Autonomous Navigation Controller
=========================================

Unified flight controller for Drone-2 sprayer missions.
Handles: ARM → TAKEOFF → NAVIGATE → WAIT/RTL cycle.

Modeled after drone1_navigation for consistent behavior.

State Machine:
    IDLE → WAIT_FCU → SET_GUIDED → ARM → TAKEOFF → NAVIGATE → 
    ARRIVED → WAIT_FOR_NEXT → [NAVIGATE or RTL] → LANDED

Subscribers:
    /drone2/target_position (sensor_msgs/NavSatFix): Target from telem_rx
    /mavros/state (mavros_msgs/State): FCU connection & mode
    /mavros/global_position/global (sensor_msgs/NavSatFix): Current GPS
    /mavros/local_position/pose (geometry_msgs/PoseStamped): Local position

Publishers:
    /mavros/setpoint_position/local (geometry_msgs/PoseStamped): Setpoints
    /drone2/arrival_status (std_msgs/Bool): Arrival at target
    /drone2/navigation_status (std_msgs/String): State for monitoring
"""

import math
from enum import Enum, auto
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, String, Header
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL


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
    WAIT_FOR_NEXT = auto()
    RTL = auto()
    LANDED = auto()


class Drone2NavigationNode(Node):
    """Unified autonomous flight controller for Drone-2."""
    
    EARTH_RADIUS = 6371000  # meters
    
    def __init__(self):
        super().__init__('drone2_navigation_node')
        
        # ================================================================
        # PARAMETERS
        # ================================================================
        self.declare_parameter('takeoff_altitude_m', 10.0)
        self.declare_parameter('arrival_radius_m', 3.0)
        self.declare_parameter('setpoint_rate_hz', 10.0)
        self.declare_parameter('wait_timeout_sec', 30.0)
        self.declare_parameter('takeoff_timeout_sec', 60.0)
        self.declare_parameter('fcu_timeout_sec', 30.0)
        
        self.takeoff_alt = self.get_parameter('takeoff_altitude_m').value
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
        self.home_gps: Optional[Tuple[float, float]] = None
        self.home_local: Optional[Tuple[float, float, float]] = None
        
        # Target
        self.target_gps: Optional[NavSatFix] = None
        self.target_local: Optional[Tuple[float, float, float]] = None
        self.targets_completed = 0
        
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
        self.create_subscription(
            NavSatFix, '/drone2/target_position',
            self.target_callback, reliable_qos
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
        
        # ================================================================
        # TIMERS
        # ================================================================
        self.create_timer(0.5, self.fsm_update)
        self.create_timer(1.0 / self.setpoint_rate, self.publish_setpoint)
        self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Drone-2 Autonomous Navigation Controller')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Takeoff: {self.takeoff_alt}m')
        self.get_logger().info(f'  Arrival radius: {self.arrival_radius}m')
        self.get_logger().info(f'  Wait timeout: {self.wait_timeout}s')
    
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
        
        # Convert to local if we have home
        if self.home_gps and self.home_local:
            self.target_local = self.gps_to_local(
                msg.latitude, msg.longitude, self.takeoff_alt
            )
            self.get_logger().info(
                f'Target local: x={self.target_local[0]:.1f}, '
                f'y={self.target_local[1]:.1f}, z={self.target_local[2]:.1f}'
            )
        
        # Trigger flight if waiting for target
        if self.state == FlightState.WAIT_TARGET:
            self.transition_to(FlightState.SET_GUIDED, 'First target received')
        elif self.state == FlightState.WAIT_FOR_NEXT:
            self.transition_to(FlightState.NAVIGATE, 'New target during wait')
    
    def state_callback(self, msg: State):
        """Monitor FCU connection and mode."""
        self.fcu_state = msg
        if msg.connected and not self.fcu_connected:
            self.fcu_connected = True
            self.get_logger().info('FCU connected')
    
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
        
        elif self.state == FlightState.WAIT_FOR_NEXT:
            self.handle_wait_for_next()
        
        elif self.state == FlightState.RTL:
            self.handle_rtl()
    
    # ====================================================================
    # STATE HANDLERS
    # ====================================================================
    
    def handle_set_guided(self):
        """Request GUIDED mode."""
        if not self.fcu_state:
            return
        
        if self.fcu_state.mode == 'GUIDED':
            self.transition_to(FlightState.ARM, 'GUIDED confirmed')
            return
        
        if self.time_in_state() < 1.0:
            return
        
        if self.mode_client.wait_for_service(timeout_sec=0.5):
            req = SetMode.Request()
            req.custom_mode = 'GUIDED'
            self.mode_client.call_async(req)
            self.get_logger().info('Requesting GUIDED mode')
    
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
        """Monitor altitude during takeoff."""
        if not self.fcu_state or not self.fcu_state.armed:
            self.get_logger().error('Disarmed during takeoff')
            self.transition_to(FlightState.RTL, 'Disarmed')
            return
        
        if self.time_in_state() > self.takeoff_timeout:
            self.get_logger().error('Takeoff timeout')
            self.transition_to(FlightState.RTL, 'Timeout')
            return
        
        if self.local_pose is None:
            return
        
        alt = self.local_pose.pose.position.z
        target = self.takeoff_alt * 0.8
        
        if int(self.time_in_state()) % 2 == 0:
            self.get_logger().info(f'Climbing: {alt:.1f}m / {self.takeoff_alt}m')
        
        if alt >= target:
            self.get_logger().info(f'Takeoff complete: {alt:.1f}m')
            self.transition_to(FlightState.NAVIGATE, 'At altitude')
    
    def handle_navigate(self):
        """Navigate to target."""
        if not self.fcu_state or not self.fcu_state.armed:
            self.get_logger().error('Disarmed during navigation')
            self.transition_to(FlightState.RTL, 'Disarmed')
            return
        
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
        """Handle arrival at target."""
        self.targets_completed += 1
        self.get_logger().info(f'Arrived! Targets completed: {self.targets_completed}')
        
        # Publish arrival
        msg = Bool()
        msg.data = True
        self.arrival_pub.publish(msg)
        
        self.transition_to(FlightState.WAIT_FOR_NEXT, 'Published arrival')
    
    def handle_wait_for_next(self):
        """Wait for next target or RTL."""
        if self.time_in_state() >= self.wait_timeout:
            self.get_logger().info(f'{self.wait_timeout}s timeout - RTL')
            self.transition_to(FlightState.RTL, 'Wait timeout')
    
    def handle_rtl(self):
        """Request RTL mode."""
        if self.time_in_state() < 0.1:
            if self.mode_client.wait_for_service(timeout_sec=0.5):
                req = SetMode.Request()
                req.custom_mode = 'RTL'
                self.mode_client.call_async(req)
                self.get_logger().info('RTL requested')
    
    # ====================================================================
    # SETPOINT PUBLISHING (10Hz)
    # ====================================================================
    
    def publish_setpoint(self):
        """Publish position setpoints continuously."""
        # Only during active flight states
        if self.state not in [
            FlightState.SET_GUIDED, FlightState.ARM,
            FlightState.NAVIGATE, FlightState.ARRIVED,
            FlightState.WAIT_FOR_NEXT
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
                           FlightState.WAIT_FOR_NEXT]:
            # Navigate to target
            if self.target_local:
                setpoint.pose.position.x = self.target_local[0]
                setpoint.pose.position.y = self.target_local[1]
                setpoint.pose.position.z = self.target_local[2]
            else:
                setpoint.pose.position.x = self.local_pose.pose.position.x
                setpoint.pose.position.y = self.local_pose.pose.position.y
                setpoint.pose.position.z = self.takeoff_alt
        
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
