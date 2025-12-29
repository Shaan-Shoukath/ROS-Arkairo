#!/usr/bin/env python3
# Copyright 2024 Maintainer
# SPDX-License-Identifier: Apache-2.0

"""
GCS to D2 Downlink Node

Converts GCS mission commands into local navigation triggers for Drone-2.
Acts as bridge between telemetry and local mission execution.

Subscribers:
    /drone2/target_geotag (geographic_msgs/GeoPointStamped): Target from GCS
    /drone2/mission_start (std_msgs/Bool): Mission start trigger

Publishers:
    /drone2/target_position (sensor_msgs/NavSatFix): Local nav target
    /drone2/new_target_received (std_msgs/Bool): New target trigger
"""

import math
from typing import Optional
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, Header
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPointStamped


class GcsToD2DownlinkNode(Node):
    """ROS2 node for GCS to Drone-2 mission bridging."""
    
    EARTH_RADIUS = 6371000  # meters
    
    def __init__(self):
        super().__init__('gcs_to_d2_downlink_node')
        
        # Declare parameters
        self.declare_parameter('require_confirmation', False)  # Auto-dispatch
        self.declare_parameter('confirmation_timeout_sec', 5.0)
        self.declare_parameter('override_altitude', True)
        self.declare_parameter('target_altitude_m', 10.0)
        self.declare_parameter('validate_coordinates', True)
        self.declare_parameter('max_distance_from_home_m', 10000.0)
        self.declare_parameter('home_latitude', 0.0)
        self.declare_parameter('home_longitude', 0.0)
        
        # Get parameters
        self.require_confirm = self.get_parameter('require_confirmation').value
        self.confirm_timeout = self.get_parameter('confirmation_timeout_sec').value
        self.override_alt = self.get_parameter('override_altitude').value
        self.target_alt = self.get_parameter('target_altitude_m').value
        self.validate_coords = self.get_parameter('validate_coordinates').value
        self.max_distance = self.get_parameter('max_distance_from_home_m').value
        self.home_lat = self.get_parameter('home_latitude').value
        self.home_lon = self.get_parameter('home_longitude').value
        
        # State
        self.pending_target: Optional[GeoPointStamped] = None
        self.pending_time: Optional[datetime] = None
        self.mission_active = False
        
        # Statistics
        self.total_received = 0
        self.total_dispatched = 0
        self.total_rejected = 0
        
        # QoS for reliable message delivery
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Subscribers
        self.geotag_sub = self.create_subscription(
            GeoPointStamped,
            '/drone2/target_geotag',
            self.geotag_callback,
            reliable_qos
        )
        
        self.start_sub = self.create_subscription(
            Bool,
            '/drone2/mission_start',
            self.start_callback,
            reliable_qos
        )
        
        # Publishers
        self.position_pub = self.create_publisher(
            NavSatFix,
            '/drone2/target_position',
            reliable_qos
        )
        
        self.trigger_pub = self.create_publisher(
            Bool,
            '/drone2/new_target_received',
            10
        )
        
        # Timeout check timer
        if self.require_confirm:
            self.timeout_timer = self.create_timer(1.0, self.check_timeout)
        
        self.get_logger().info('GCS to D2 Downlink Node initialized')
        self.get_logger().info(f'  Require confirmation: {self.require_confirm}')
        self.get_logger().info(f'  Override altitude: {self.override_alt}')
    
    def geotag_callback(self, msg: GeoPointStamped):
        """Receive target geotag from GCS."""
        self.total_received += 1
        
        lat = msg.position.latitude
        lon = msg.position.longitude
        alt = msg.position.altitude
        
        self.get_logger().info(
            f'Received target from GCS: lat={lat:.6f}, lon={lon:.6f}'
        )
        
        # Validate coordinates
        if self.validate_coords and not self._validate_coordinates(lat, lon):
            self.total_rejected += 1
            self.get_logger().error('Target rejected: invalid coordinates')
            return
        
        # Store pending target
        if self.require_confirm:
            self.pending_target = msg
            self.pending_time = datetime.now()
            self.get_logger().info('Target pending - awaiting start confirmation')
        else:
            # Direct dispatch
            self._dispatch_target(msg)
    
    def start_callback(self, msg: Bool):
        """Receive mission start trigger."""
        if not msg.data:
            return
        
        if self.pending_target is None:
            self.get_logger().warn('Start received but no pending target')
            return
        
        self.get_logger().info('Mission start confirmed')
        self._dispatch_target(self.pending_target)
        self.pending_target = None
        self.pending_time = None
    
    def _dispatch_target(self, geotag: GeoPointStamped):
        """Dispatch target to navigation system."""
        # Create NavSatFix message
        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.latitude = geotag.position.latitude
        msg.longitude = geotag.position.longitude
        
        # Override altitude if configured
        if self.override_alt:
            msg.altitude = self.target_alt
        else:
            msg.altitude = geotag.position.altitude
        
        msg.status.status = 0  # STATUS_FIX
        msg.status.service = 1  # SERVICE_GPS
        
        self.position_pub.publish(msg)
        
        # Send trigger
        trigger_msg = Bool()
        trigger_msg.data = True
        self.trigger_pub.publish(trigger_msg)
        
        self.mission_active = True
        self.total_dispatched += 1
        
        self.get_logger().info(
            f'Dispatched target: lat={msg.latitude:.6f}, '
            f'lon={msg.longitude:.6f}, alt={msg.altitude:.1f}m'
        )
    
    def _validate_coordinates(self, lat: float, lon: float) -> bool:
        """Validate GPS coordinates."""
        # Basic bounds check
        if lat < -90 or lat > 90:
            return False
        if lon < -180 or lon > 180:
            return False
        
        # Reject null island
        if lat == 0.0 and lon == 0.0:
            return False
        
        # Check distance from home if home is set
        if self.home_lat != 0.0 or self.home_lon != 0.0:
            distance = self._haversine_distance(
                lat, lon, self.home_lat, self.home_lon
            )
            if distance > self.max_distance:
                self.get_logger().warn(
                    f'Target {distance:.0f}m from home, '
                    f'exceeds max {self.max_distance:.0f}m'
                )
                return False
        
        return True
    
    def _haversine_distance(
        self,
        lat1: float, lon1: float,
        lat2: float, lon2: float
    ) -> float:
        """Calculate distance between two GPS points."""
        lat1, lon1 = math.radians(lat1), math.radians(lon1)
        lat2, lon2 = math.radians(lat2), math.radians(lon2)
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = math.sin(dlat/2)**2 + \
            math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        
        return self.EARTH_RADIUS * c
    
    def check_timeout(self):
        """Check for pending target timeout."""
        if self.pending_target is None:
            return
        
        elapsed = (datetime.now() - self.pending_time).total_seconds()
        
        if elapsed > self.confirm_timeout:
            self.get_logger().warn(
                f'Pending target timed out after {elapsed:.1f}s'
            )
            self.pending_target = None
            self.pending_time = None


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = GcsToD2DownlinkNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Statistics: received={node.total_received}, '
            f'dispatched={node.total_dispatched}, '
            f'rejected={node.total_rejected}'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
