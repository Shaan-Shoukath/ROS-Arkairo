#!/usr/bin/env python3
# Copyright 2024 Shaan Shoukath
# SPDX-License-Identifier: Apache-2.0

"""
Telemetry RX Node

Receives disease geotags from MAVLink telemetry (via GCS or direct), validates
them, and dispatches to the navigation system for Drone-2.

Supports two decoding modes:
    1. STATUSTEXT - Single message with "GEOTAG:lat,lon,alt" format
    2. NAMED_VALUE_FLOAT - Three separate messages (d_lat, d_lon, d_alt)

Subscribers:
    /mavros/statustext/recv (mavros_msgs/StatusText): STATUSTEXT mode
    /mavros/debug_value/recv (mavros_msgs/DebugValue): NAMED_VALUE_FLOAT mode

Publishers:
    /drone2/target_position (sensor_msgs/NavSatFix): Navigation target
    /drone2/new_target_received (std_msgs/Bool): Trigger for nav node
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geographic_msgs.msg import GeoPoint
from mavros_msgs.msg import DebugValue, StatusText
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header, Bool


class TelemRxNode(Node):
    """ROS2 node for receiving and dispatching disease geotags from telemetry."""
    
    EARTH_RADIUS = 6371000  # meters
    GEOTAG_PREFIX = "GEOTAG:"
    
    def __init__(self):
        super().__init__('telem_rx_node')
        
        # Mode parameter
        self.declare_parameter('use_statustext', True)  # True = STATUSTEXT, False = NAMED_VALUE_FLOAT
        self.use_statustext = self.get_parameter('use_statustext').value
        
        # Buffer parameters
        self.declare_parameter('buffer_timeout_sec', 5.0)
        self.buffer_timeout = self.get_parameter('buffer_timeout_sec').value
        
        # Validation parameters
        self.declare_parameter('validate_coordinates', True)
        self.declare_parameter('max_distance_from_home_m', 10000.0)
        self.declare_parameter('home_latitude', 0.0)
        self.declare_parameter('home_longitude', 0.0)
        
        # Altitude override parameters
        self.declare_parameter('override_altitude', False)
        self.declare_parameter('target_altitude_m', 20.0)
        
        # Dummy geotag parameters for testing
        self.declare_parameter('use_dummy_geotags', False)
        self.declare_parameter('dummy_lat', 10.0480)
        self.declare_parameter('dummy_lon', 76.3305)
        self.declare_parameter('dummy_alt', 10.0)
        self.declare_parameter('dummy_interval_sec', 10.0)
        
        # Get parameters
        self.validate_coords = self.get_parameter('validate_coordinates').value
        self.max_distance = self.get_parameter('max_distance_from_home_m').value
        self.home_lat = self.get_parameter('home_latitude').value
        self.home_lon = self.get_parameter('home_longitude').value
        self.override_alt = self.get_parameter('override_altitude').value
        self.target_alt = self.get_parameter('target_altitude_m').value
        
        # Dummy mode parameters
        self.use_dummy = self.get_parameter('use_dummy_geotags').value
        self.dummy_lat = self.get_parameter('dummy_lat').value
        self.dummy_lon = self.get_parameter('dummy_lon').value
        self.dummy_alt = self.get_parameter('dummy_alt').value
        self.dummy_interval = self.get_parameter('dummy_interval_sec').value
        
        # Message buffer (for NAMED_VALUE_FLOAT mode)
        self.buffer_lat: Optional[float] = None
        self.buffer_lon: Optional[float] = None
        self.buffer_alt: Optional[float] = None
        self.buffer_stamp: Optional[float] = None
        self.last_buffer_time: Optional[float] = None
        
        # Duplicate detection
        self.last_geotag: Optional[str] = None
        
        # Statistics
        self.rx_count = 0
        self.dispatched_count = 0
        self.rejected_count = 0
        self.duplicate_count = 0
        self.dummy_sent = False
        
        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        target_pos_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Subscribers based on mode
        if not self.use_dummy:
            if self.use_statustext:
                self.statustext_sub = self.create_subscription(
                    StatusText,
                    '/mavros/statustext/recv',
                    self.statustext_callback,
                    10
                )
                self.get_logger().info('Mode: STATUSTEXT')
            else:
                self.debug_sub = self.create_subscription(
                    DebugValue,
                    '/mavros/debug_value/recv',
                    self.debug_callback,
                    10
                )
                self.get_logger().info('Mode: NAMED_VALUE_FLOAT')
        
        # Publishers
        self.position_pub = self.create_publisher(
            NavSatFix,
            '/drone2/target_position',
            target_pos_qos
        )
        
        self.trigger_pub = self.create_publisher(
            Bool,
            '/drone2/new_target_received',
            10
        )
        
        # Timers
        if not self.use_dummy and not self.use_statustext:
            self.timeout_timer = self.create_timer(1.0, self.check_buffer_timeout)
        
        if self.use_dummy:
            self.dummy_timer = self.create_timer(self.dummy_interval, self.publish_dummy_geotag)
            self.get_logger().info('=== DUMMY GEOTAG MODE ENABLED ===')
            self.get_logger().info(f'  Target: lat={self.dummy_lat:.6f}, lon={self.dummy_lon:.6f}')
        else:
            self.get_logger().info('Telemetry RX Node initialized')
        
        self.get_logger().info('  Publishing: /drone2/target_position')
    
    def statustext_callback(self, msg: StatusText):
        """Handle STATUSTEXT messages containing geotags."""
        text = msg.text.strip()
        
        # Only process GEOTAG messages
        if not text.startswith(self.GEOTAG_PREFIX):
            return
        
        # Duplicate check
        if text == self.last_geotag:
            self.duplicate_count += 1
            self.get_logger().debug(f'Duplicate geotag ignored: {text}')
            return
        
        self.last_geotag = text
        self.rx_count += 1
        
        # Parse: GEOTAG:lat,lon,alt
        try:
            coords = text[len(self.GEOTAG_PREFIX):].split(',')
            lat = float(coords[0])
            lon = float(coords[1])
            alt = float(coords[2]) if len(coords) > 2 else 10.0
        except (ValueError, IndexError) as e:
            self.get_logger().error(f'Failed to parse geotag: {text} - {e}')
            self.rejected_count += 1
            return
        
        self.get_logger().info(f'RX #{self.rx_count}: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}')
        
        # Validate and dispatch
        if self.validate_coords and not self._validate_coordinates(lat, lon):
            self.rejected_count += 1
            self.get_logger().error('Target rejected: invalid coordinates')
            return
        
        self._dispatch_target(lat, lon, alt)
    
    def debug_callback(self, msg: DebugValue):
        """Handle NAMED_VALUE_FLOAT messages (buffer and reconstruct)."""
        if msg.type != DebugValue.TYPE_NAMED_VALUE_FLOAT:
            return
        
        name = msg.name.strip()
        if name not in ('d_lat', 'd_lon', 'd_alt'):
            return
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        msg_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        
        # New message group check
        if self.buffer_stamp is not None and abs(msg_stamp - self.buffer_stamp) > 0.1:
            self._clear_buffer()
        
        self.buffer_stamp = msg_stamp
        self.last_buffer_time = current_time
        
        if name == 'd_lat':
            self.buffer_lat = msg.value_float
        elif name == 'd_lon':
            self.buffer_lon = msg.value_float
        elif name == 'd_alt':
            self.buffer_alt = msg.value_float
        
        if self._buffer_complete():
            self._process_buffered_geotag()
            self._clear_buffer()
    
    def _buffer_complete(self) -> bool:
        """Check if all buffer components are present."""
        return all([
            self.buffer_lat is not None,
            self.buffer_lon is not None,
            self.buffer_alt is not None
        ])
    
    def _process_buffered_geotag(self):
        """Process completed buffered geotag."""
        self.rx_count += 1
        lat, lon, alt = self.buffer_lat, self.buffer_lon, self.buffer_alt
        
        self.get_logger().info(f'RX #{self.rx_count}: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}')
        
        if self.validate_coords and not self._validate_coordinates(lat, lon):
            self.rejected_count += 1
            self.get_logger().error('Target rejected: invalid coordinates')
            return
        
        self._dispatch_target(lat, lon, alt)
    
    def _clear_buffer(self):
        """Clear message buffer."""
        self.buffer_lat = None
        self.buffer_lon = None
        self.buffer_alt = None
        self.buffer_stamp = None
    
    def publish_dummy_geotag(self):
        """Publish dummy geotag for testing."""
        if self.dummy_sent:
            return
        
        self.get_logger().info('Publishing DUMMY geotag...')
        
        if self.validate_coords and not self._validate_coordinates(self.dummy_lat, self.dummy_lon):
            self.get_logger().error('Dummy rejected: invalid coordinates')
            return
        
        self._dispatch_target(self.dummy_lat, self.dummy_lon, self.dummy_alt)
        self.dummy_sent = True
    
    def _validate_coordinates(self, lat: float, lon: float) -> bool:
        """Validate GPS coordinates."""
        if lat < -90 or lat > 90:
            return False
        if lon < -180 or lon > 180:
            return False
        if lat == 0.0 and lon == 0.0:
            return False
        
        if self.home_lat != 0.0 or self.home_lon != 0.0:
            distance = self._haversine_distance(lat, lon, self.home_lat, self.home_lon)
            if distance > self.max_distance:
                self.get_logger().warn(f'Target {distance:.0f}m from home, max={self.max_distance:.0f}m')
                return False
        
        return True
    
    def _haversine_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate distance between two GPS points."""
        lat1, lon1 = math.radians(lat1), math.radians(lon1)
        lat2, lon2 = math.radians(lat2), math.radians(lon2)
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        
        return self.EARTH_RADIUS * c
    
    def _dispatch_target(self, lat: float, lon: float, alt: float):
        """Dispatch target to navigation."""
        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = self.target_alt if self.override_alt else alt
        
        msg.status.status = 0
        msg.status.service = 1
        
        self.position_pub.publish(msg)
        
        trigger_msg = Bool()
        trigger_msg.data = True
        self.trigger_pub.publish(trigger_msg)
        
        self.dispatched_count += 1
        self.get_logger().info(f'Dispatched #{self.dispatched_count}: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}')
    
    def check_buffer_timeout(self):
        """Clear buffer on timeout."""
        if self.last_buffer_time is None:
            return
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.last_buffer_time
        
        if elapsed > self.buffer_timeout and not self._buffer_complete():
            self.get_logger().warn(f'Buffer timeout ({elapsed:.1f}s) - clearing')
            self._clear_buffer()
            self.last_buffer_time = None


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = TelemRxNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Stats: rx={node.rx_count}, dispatched={node.dispatched_count}, '
            f'rejected={node.rejected_count}, duplicates={node.duplicate_count}'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
