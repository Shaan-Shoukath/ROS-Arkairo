#!/usr/bin/env python3
# Copyright 2024 Maintainer
# SPDX-License-Identifier: Apache-2.0

"""
GCS Mission Router Node

Controls mission dispatch to Drone-2 based on validated targets and
drone status. Manages target queue and mission sequencing.

Subscribers:
    /gcs/validated_target (geographic_msgs/GeoPointStamped): Validated targets
    /drone2/status (std_msgs/String): Drone-2 operational status

Publishers:
    /drone2/target_geotag (geographic_msgs/GeoPointStamped): Target for Drone-2
    /drone2/mission_start (std_msgs/Bool): Mission start trigger
"""

from enum import Enum, auto
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, String, Header
from geographic_msgs.msg import GeoPointStamped, GeoPoint


class Drone2Status(Enum):
    """Drone-2 operational status."""
    UNKNOWN = auto()
    OFFLINE = auto()
    IDLE = auto()
    NAVIGATING = auto()
    DETECTING = auto()
    CENTERING = auto()
    SPRAYING = auto()
    RETURNING = auto()
    ERROR = auto()


@dataclass
class PendingTarget:
    """Target awaiting dispatch."""
    geotag: GeoPointStamped
    received_time: datetime
    target_id: int


class GcsMissionRouterNode(Node):
    """ROS2 node for mission dispatch to Drone-2."""
    
    def __init__(self):
        super().__init__('gcs_mission_router_node')
        
        # Declare parameters
        self.declare_parameter('auto_dispatch', True)
        self.declare_parameter('dispatch_delay_sec', 2.0)
        self.declare_parameter('max_pending_targets', 50)
        self.declare_parameter('fifo_priority', True)
        self.declare_parameter('drone2_timeout_sec', 30.0)
        self.declare_parameter('require_idle_status', True)
        self.declare_parameter('default_spray_duration_sec', 5.0)
        self.declare_parameter('default_approach_altitude_m', 10.0)
        self.declare_parameter('max_missions_per_hour', 60)
        self.declare_parameter('mission_cooldown_sec', 10.0)
        
        # Get parameters
        self.auto_dispatch = self.get_parameter('auto_dispatch').value
        self.dispatch_delay = self.get_parameter('dispatch_delay_sec').value
        self.max_pending = self.get_parameter('max_pending_targets').value
        self.fifo = self.get_parameter('fifo_priority').value
        self.drone2_timeout = self.get_parameter('drone2_timeout_sec').value
        self.require_idle = self.get_parameter('require_idle_status').value
        self.spray_duration = self.get_parameter('default_spray_duration_sec').value
        self.approach_alt = self.get_parameter('default_approach_altitude_m').value
        self.max_missions = self.get_parameter('max_missions_per_hour').value
        self.cooldown = self.get_parameter('mission_cooldown_sec').value
        
        # State
        self.target_queue: Deque[PendingTarget] = deque(maxlen=self.max_pending)
        self.drone2_status = Drone2Status.UNKNOWN
        self.last_drone2_update = None
        self.last_dispatch_time = None
        self.current_mission_target: Optional[PendingTarget] = None
        self.next_target_id = 1
        
        # Statistics
        self.total_received = 0
        self.total_dispatched = 0
        self.total_completed = 0
        self.total_failed = 0
        self.missions_this_hour = 0
        self.hour_start_time = datetime.now()
        
        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Subscribers
        self.target_sub = self.create_subscription(
            GeoPointStamped,
            '/gcs/validated_target',
            self.target_callback,
            reliable_qos
        )
        
        self.status_sub = self.create_subscription(
            String,
            '/drone2/status',
            self.status_callback,
            10
        )
        
        # Publishers
        self.geotag_pub = self.create_publisher(
            GeoPointStamped,
            '/drone2/target_geotag',
            reliable_qos
        )
        
        self.mission_pub = self.create_publisher(
            Bool,
            '/drone2/mission_start',
            reliable_qos
        )
        
        # Dispatch timer
        self.dispatch_timer = self.create_timer(1.0, self.dispatch_loop)
        
        # Status check timer
        self.status_timer = self.create_timer(5.0, self.check_drone2_status)
        
        self.get_logger().info('GCS Mission Router Node initialized')
        self.get_logger().info(f'  Auto dispatch: {self.auto_dispatch}')
        self.get_logger().info(f'  Max pending targets: {self.max_pending}')
    
    def target_callback(self, msg: GeoPointStamped):
        """Receive validated target for dispatch."""
        self.total_received += 1
        
        target = PendingTarget(
            geotag=msg,
            received_time=datetime.now(),
            target_id=self.next_target_id
        )
        self.next_target_id += 1
        
        if len(self.target_queue) >= self.max_pending:
            self.get_logger().warn('Target queue full, dropping oldest')
        
        self.target_queue.append(target)
        
        self.get_logger().info(
            f'Queued target #{target.target_id}: '
            f'lat={msg.position.latitude:.6f}, '
            f'lon={msg.position.longitude:.6f} '
            f'(queue size: {len(self.target_queue)})'
        )
    
    def status_callback(self, msg: String):
        """Update Drone-2 status."""
        self.last_drone2_update = datetime.now()
        
        status_str = msg.data.upper()
        
        # Parse status
        status_map = {
            'IDLE': Drone2Status.IDLE,
            'READY': Drone2Status.IDLE,
            'NAVIGATING': Drone2Status.NAVIGATING,
            'DETECTING': Drone2Status.DETECTING,
            'CENTERING': Drone2Status.CENTERING,
            'SPRAYING': Drone2Status.SPRAYING,
            'RETURNING': Drone2Status.RETURNING,
            'RTL': Drone2Status.RETURNING,
            'ERROR': Drone2Status.ERROR,
            'FAULT': Drone2Status.ERROR,
        }
        
        new_status = status_map.get(status_str, Drone2Status.UNKNOWN)
        
        # Check for mission completion
        if (self.current_mission_target and 
            self.drone2_status != Drone2Status.IDLE and
            new_status == Drone2Status.IDLE):
            self._handle_mission_complete()
        
        if new_status != self.drone2_status:
            self.get_logger().info(
                f'Drone-2 status: {self.drone2_status.name} -> {new_status.name}'
            )
            self.drone2_status = new_status
    
    def dispatch_loop(self):
        """Main dispatch control loop."""
        if not self.auto_dispatch:
            return
        
        if not self.target_queue:
            return
        
        if not self._can_dispatch():
            return
        
        # Get next target
        target = self.target_queue.popleft()
        
        # Dispatch to Drone-2
        self._dispatch_target(target)
    
    def _can_dispatch(self) -> bool:
        """Check if we can dispatch a mission."""
        # Check Drone-2 status
        if self.drone2_status == Drone2Status.OFFLINE:
            return False
        
        if self.drone2_status == Drone2Status.ERROR:
            return False
        
        if self.require_idle and self.drone2_status != Drone2Status.IDLE:
            return False
        
        # Check cooldown
        if self.last_dispatch_time:
            elapsed = (datetime.now() - self.last_dispatch_time).total_seconds()
            if elapsed < self.cooldown:
                return False
        
        # Check hourly limit
        self._update_hourly_counter()
        if self.missions_this_hour >= self.max_missions:
            self.get_logger().warn('Hourly mission limit reached')
            return False
        
        return True
    
    def _dispatch_target(self, target: PendingTarget):
        """Dispatch target to Drone-2."""
        self.get_logger().info(
            f'Dispatching target #{target.target_id} to Drone-2'
        )
        
        # Publish target geotag
        geotag_msg = GeoPointStamped()
        geotag_msg.header = Header()
        geotag_msg.header.stamp = self.get_clock().now().to_msg()
        geotag_msg.header.frame_id = 'map'
        geotag_msg.position = target.geotag.position
        
        self.geotag_pub.publish(geotag_msg)
        
        # Wait briefly then send start trigger
        self.create_timer(
            self.dispatch_delay,
            lambda: self._send_start_trigger(target),
            callback_group=None
        )
    
    def _send_start_trigger(self, target: PendingTarget):
        """Send mission start trigger."""
        start_msg = Bool()
        start_msg.data = True
        self.mission_pub.publish(start_msg)
        
        self.current_mission_target = target
        self.last_dispatch_time = datetime.now()
        self.total_dispatched += 1
        self.missions_this_hour += 1
        
        self.get_logger().info(
            f'Mission started for target #{target.target_id}'
        )
    
    def _handle_mission_complete(self):
        """Handle mission completion."""
        if self.current_mission_target:
            self.total_completed += 1
            self.get_logger().info(
                f'Mission completed for target #{self.current_mission_target.target_id}'
            )
            self.current_mission_target = None
    
    def _update_hourly_counter(self):
        """Reset hourly counter if hour has passed."""
        now = datetime.now()
        elapsed = (now - self.hour_start_time).total_seconds()
        
        if elapsed >= 3600:
            self.missions_this_hour = 0
            self.hour_start_time = now
    
    def check_drone2_status(self):
        """Check if Drone-2 is still online."""
        if self.last_drone2_update is None:
            self.drone2_status = Drone2Status.OFFLINE
            return
        
        elapsed = (datetime.now() - self.last_drone2_update).total_seconds()
        
        if elapsed > self.drone2_timeout:
            if self.drone2_status != Drone2Status.OFFLINE:
                self.get_logger().warn('Drone-2 status timeout - marking offline')
                self.drone2_status = Drone2Status.OFFLINE


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = GcsMissionRouterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Statistics: received={node.total_received}, '
            f'dispatched={node.total_dispatched}, '
            f'completed={node.total_completed}'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
