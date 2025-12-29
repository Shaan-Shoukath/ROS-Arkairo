#!/usr/bin/env python3
# Copyright 2024 Maintainer
# SPDX-License-Identifier: Apache-2.0

"""
Drone-2 Mission Manager Node - FULLY AUTONOMOUS

Manages complete mission lifecycle with auto-arm, spray cycles, and RTL.

State Machine:
    IDLE → ARMING → TAKING_OFF → NAVIGATING → DETECTING → CENTERING
    → SPRAYING → WAITING_FOR_NEXT → [NAVIGATING or RETURNING_HOME]
    → LANDED → IDLE

Subscribers:
    /drone2/target_geotag (geographic_msgs/GeoPointStamped): Target from GCS
    /drone2/arrival_status (std_msgs/Bool): Arrived at target
    /drone2/local_detection_status (std_msgs/Bool): Detection confirmed
    /drone2/spray_ready (std_msgs/Bool): Centering complete
    /drone2/spray_done (std_msgs/Bool): Spray complete
    /mavros/state (mavros_msgs/State): Flight controller state
    /mavros/local_position/pose (geometry_msgs/PoseStamped): Current pose

Publishers:
    /drone2/status (std_msgs/String): Current state
    /drone2/target_position (sensor_msgs/NavSatFix): Navigation target
    /drone2/new_target_received (std_msgs/Bool): Nav trigger
"""

from enum import Enum, auto
from typing import Optional
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import Bool, String, Header
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL


class MissionState(Enum):
    """Drone-2 mission states."""
    IDLE = auto()              # Disarmed, waiting for first geotag
    ARMING = auto()            # Requesting arm
    TAKING_OFF = auto()        # Climbing to altitude
    NAVIGATING = auto()        # Flying to target
    DETECTING = auto()         # Local detection
    CENTERING = auto()         # Visual servoing
    SPRAYING = auto()          # Spray active
    WAITING_FOR_NEXT = auto()  # 5 second wait window
    RETURNING_HOME = auto()    # RTL
    LANDED = auto()            # On ground, disarmed


@dataclass
class PendingTarget:
    """Target awaiting processing."""
    geotag: GeoPointStamped
    received_time: float


class MissionManagerNode(Node):
    """Fully autonomous Drone-2 mission manager."""
    
    def __init__(self):
        super().__init__('mission_manager_node')
        
        # Callback group for services
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        
        # Parameters
        self.declare_parameter('wait_timeout_sec', 5.0)
        self.declare_parameter('takeoff_altitude_m', 10.0)
        self.declare_parameter('arming_timeout_sec', 30.0)
        self.declare_parameter('takeoff_timeout_sec', 60.0)
        self.declare_parameter('altitude_tolerance_m', 1.5)
        self.declare_parameter('status_rate_hz', 2.0)
        
        self.wait_timeout = self.get_parameter('wait_timeout_sec').value
        self.takeoff_alt = self.get_parameter('takeoff_altitude_m').value
        self.arming_timeout = self.get_parameter('arming_timeout_sec').value
        self.takeoff_timeout = self.get_parameter('takeoff_timeout_sec').value
        self.alt_tolerance = self.get_parameter('altitude_tolerance_m').value
        status_rate = self.get_parameter('status_rate_hz').value
        
        # State
        self.state = MissionState.IDLE
        self.fc_state: Optional[State] = None
        self.current_pose: Optional[PoseStamped] = None
        self.pending_target: Optional[PendingTarget] = None
        self.current_target: Optional[GeoPointStamped] = None
        self.state_start_time: Optional[float] = None
        self.first_geotag_received = False
        
        # Statistics
        self.missions_completed = 0
        self.missions_failed = 0
        
        # QoS
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # Subscribers
        self.geotag_sub = self.create_subscription(
            GeoPointStamped, '/drone2/target_geotag',
            self.geotag_callback, reliable_qos
        )
        self.arrival_sub = self.create_subscription(
            Bool, '/drone2/arrival_status',
            self.arrival_callback, reliable_qos
        )
        self.detection_sub = self.create_subscription(
            Bool, '/drone2/local_detection_status',
            self.detection_callback, reliable_qos
        )
        self.ready_sub = self.create_subscription(
            Bool, '/drone2/spray_ready',
            self.ready_callback, reliable_qos
        )
        self.done_sub = self.create_subscription(
            Bool, '/drone2/spray_done',
            self.done_callback, reliable_qos
        )
        self.state_sub = self.create_subscription(
            State, '/mavros/state',
            self.state_callback, sensor_qos
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.pose_callback, sensor_qos
        )
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/drone2/status', 10)
        self.position_pub = self.create_publisher(
            NavSatFix, '/drone2/target_position', reliable_qos
        )
        self.trigger_pub = self.create_publisher(
            Bool, '/drone2/new_target_received', 10
        )
        
        # MAVROS services
        self.arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming',
            callback_group=self.service_cb_group
        )
        self.mode_client = self.create_client(
            SetMode, '/mavros/set_mode',
            callback_group=self.service_cb_group
        )
        self.takeoff_client = self.create_client(
            CommandTOL, '/mavros/cmd/takeoff',
            callback_group=self.service_cb_group
        )
        
        # Timers
        self.main_timer = self.create_timer(0.1, self.main_loop)
        self.status_timer = self.create_timer(1.0 / status_rate, self.publish_status)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('Drone-2 Mission Manager - FULLY AUTONOMOUS')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'  Wait timeout: {self.wait_timeout}s')
        self.get_logger().info(f'  Takeoff altitude: {self.takeoff_alt}m')
        self.get_logger().info('  State: IDLE - Waiting for first geotag')
    
    def _transition(self, new_state: MissionState):
        """Transition to new state."""
        old = self.state
        self.state = new_state
        self.state_start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f'State: {old.name} → {new_state.name}')
    
    def _elapsed_time(self) -> float:
        """Time since state entry."""
        if self.state_start_time is None:
            return 0.0
        return self.get_clock().now().nanoseconds / 1e9 - self.state_start_time
    
    # === CALLBACKS ===
    
    def geotag_callback(self, msg: GeoPointStamped):
        """Receive geotag from GCS."""
        self.get_logger().info(
            f'Geotag received: ({msg.position.latitude:.6f}, '
            f'{msg.position.longitude:.6f})'
        )
        
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.pending_target = PendingTarget(geotag=msg, received_time=timestamp)
        
        # First geotag triggers arming
        if self.state == MissionState.IDLE and not self.first_geotag_received:
            self.first_geotag_received = True
            self.get_logger().info('First geotag! Starting autonomous sequence')
            self._transition(MissionState.ARMING)
            self._request_guided_and_arm()
        
        # If waiting for next, immediately process
        elif self.state == MissionState.WAITING_FOR_NEXT:
            self.get_logger().info('New geotag during wait - processing')
            self._dispatch_target(msg)
    
    def arrival_callback(self, msg: Bool):
        """Handle arrival at target."""
        if msg.data and self.state == MissionState.NAVIGATING:
            self._transition(MissionState.DETECTING)
    
    def detection_callback(self, msg: Bool):
        """Handle detection result."""
        if self.state == MissionState.DETECTING:
            if msg.data:
                self._transition(MissionState.CENTERING)
            else:
                self.get_logger().warn('Detection failed - skipping')
                self._start_wait_or_rtl()
    
    def ready_callback(self, msg: Bool):
        """Handle centering complete."""
        if msg.data and self.state == MissionState.CENTERING:
            self._transition(MissionState.SPRAYING)
    
    def done_callback(self, msg: Bool):
        """Handle spray complete."""
        if self.state == MissionState.SPRAYING:
            if msg.data:
                self.missions_completed += 1
                self.get_logger().info(
                    f'Spray complete! Total: {self.missions_completed}'
                )
            else:
                self.missions_failed += 1
            self._start_wait_or_rtl()
    
    def state_callback(self, msg: State):
        """Update FC state."""
        self.fc_state = msg
    
    def pose_callback(self, msg: PoseStamped):
        """Update pose."""
        self.current_pose = msg
    
    # === MAIN LOOP ===
    
    def main_loop(self):
        """Main state machine loop."""
        
        if self.state == MissionState.IDLE:
            return
        
        if self.state == MissionState.ARMING:
            self._handle_arming()
        
        elif self.state == MissionState.TAKING_OFF:
            self._handle_takeoff()
        
        elif self.state == MissionState.WAITING_FOR_NEXT:
            self._handle_waiting()
        
        elif self.state == MissionState.RETURNING_HOME:
            self._handle_rtl()
        
        elif self.state == MissionState.LANDED:
            self._handle_landed()
    
    def _handle_arming(self):
        """Monitor arming."""
        if self.fc_state and self.fc_state.armed:
            self.get_logger().info('ARMED - requesting takeoff')
            self._transition(MissionState.TAKING_OFF)
            self._request_takeoff()
            return
        
        if self._elapsed_time() > self.arming_timeout:
            self.get_logger().error('Arming timeout!')
            self._transition(MissionState.IDLE)
            self.first_geotag_received = False
    
    def _handle_takeoff(self):
        """Monitor takeoff."""
        if self.current_pose:
            alt = self.current_pose.pose.position.z
            if alt >= self.takeoff_alt - self.alt_tolerance:
                self.get_logger().info(f'At altitude {alt:.1f}m')
                # Dispatch pending target
                if self.pending_target:
                    self._dispatch_target(self.pending_target.geotag)
                return
        
        if self._elapsed_time() > self.takeoff_timeout:
            self.get_logger().error('Takeoff timeout - RTL')
            self._request_rtl()
    
    def _handle_waiting(self):
        """Handle 5-second wait window."""
        elapsed = self._elapsed_time()
        
        # Check for new target (handled in callback)
        if self.pending_target:
            received = self.pending_target.received_time
            if received > self.state_start_time:
                # New target arrived during wait
                self._dispatch_target(self.pending_target.geotag)
                return
        
        # Timeout - RTL
        if elapsed >= self.wait_timeout:
            self.get_logger().info(f'{self.wait_timeout}s timeout - returning home')
            self._request_rtl()
    
    def _handle_rtl(self):
        """Monitor RTL."""
        if self.current_pose:
            alt = self.current_pose.pose.position.z
            if alt < 0.5:
                if self.fc_state and not self.fc_state.armed:
                    self._transition(MissionState.LANDED)
    
    def _handle_landed(self):
        """Handle landed state."""
        self.get_logger().info('Landed - mission cycle complete')
        self.first_geotag_received = False
        self._transition(MissionState.IDLE)
    
    # === HELPERS ===
    
    def _start_wait_or_rtl(self):
        """Start wait window after spray."""
        self.get_logger().info(f'Starting {self.wait_timeout}s wait for next geotag')
        self._transition(MissionState.WAITING_FOR_NEXT)
    
    def _dispatch_target(self, geotag: GeoPointStamped):
        """Dispatch target to navigation."""
        self.current_target = geotag
        
        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.latitude = geotag.position.latitude
        msg.longitude = geotag.position.longitude
        msg.altitude = self.takeoff_alt
        
        self.position_pub.publish(msg)
        
        trigger = Bool()
        trigger.data = True
        self.trigger_pub.publish(trigger)
        
        self._transition(MissionState.NAVIGATING)
        self.pending_target = None
    
    # === MAVROS SERVICES ===
    
    def _request_guided_and_arm(self):
        """Request GUIDED mode and arm."""
        self.get_logger().info('Requesting GUIDED mode...')
        
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Mode service unavailable')
            return
        
        req = SetMode.Request()
        req.custom_mode = 'GUIDED'
        future = self.mode_client.call_async(req)
        future.add_done_callback(self._mode_done)
    
    def _mode_done(self, future):
        try:
            if future.result().mode_sent:
                self.get_logger().info('GUIDED set - arming...')
                self._request_arm()
        except Exception as e:
            self.get_logger().error(f'Mode failed: {e}')
    
    def _request_arm(self):
        if not self.arming_client.wait_for_service(timeout_sec=5.0):
            return
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info('Arm command sent')
        )
    
    def _request_takeoff(self):
        self.get_logger().info(f'Takeoff to {self.takeoff_alt}m')
        if not self.takeoff_client.wait_for_service(timeout_sec=5.0):
            return
        req = CommandTOL.Request()
        req.altitude = float(self.takeoff_alt)
        future = self.takeoff_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info('Takeoff command sent')
        )
    
    def _request_rtl(self):
        self.get_logger().info('Requesting RTL...')
        self._transition(MissionState.RETURNING_HOME)
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            return
        req = SetMode.Request()
        req.custom_mode = 'RTL'
        self.mode_client.call_async(req)
    
    def publish_status(self):
        """Publish current state."""
        msg = String()
        msg.data = self.state.name
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Final: completed={node.missions_completed}, '
            f'failed={node.missions_failed}'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
