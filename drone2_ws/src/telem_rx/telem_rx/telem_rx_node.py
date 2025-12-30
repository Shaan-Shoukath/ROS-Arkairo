#!/usr/bin/env python3
# Copyright 2024 Shaan Shoukath
# SPDX-License-Identifier: Apache-2.0

"""
Telemetry RX Node

Receives disease geotags from MAVLink telemetry, validates them, and dispatches
to the navigation system for Drone-2.

Subscribers:
    /mavros/debug_value/recv (mavros_msgs/DebugValue): MAVLink transport

Publishers:
    /drone2/target_position (sensor_msgs/NavSatFix): Navigation target
    /drone2/new_target_received (std_msgs/Bool): Trigger for nav node

Decoding:
    d_lat -> latitude
    d_lon -> longitude
    d_alt -> altitude
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geographic_msgs.msg import GeoPoint
from mavros_msgs.msg import DebugValue
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header, Bool


class TelemRxNode(Node):
    """ROS2 node for receiving and dispatching disease geotags from telemetry."""
    
    EARTH_RADIUS = 6371000  # meters
    
    def __init__(self):
        super().__init__('telem_rx_node')
        
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
        
        # Dummy geotag parameters for testing (like drone1's test mode)
        self.declare_parameter('use_dummy_geotags', False)
        self.declare_parameter('dummy_lat', 10.0480)  # Default test location
        self.declare_parameter('dummy_lon', 76.3305)
        self.declare_parameter('dummy_alt', 10.0)
        self.declare_parameter('dummy_interval_sec', 10.0)  # How often to send dummy
        
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
        
        # Message buffer
        self.buffer_lat: Optional[float] = None
        self.buffer_lon: Optional[float] = None
        self.buffer_alt: Optional[float] = None
        self.buffer_stamp: Optional[float] = None
        self.last_buffer_time: Optional[float] = None
        
        # Statistics
        self.rx_count = 0
        self.dispatched_count = 0
        self.rejected_count = 0
        self.dummy_sent = False  # Track if dummy has been sent
        
        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Subscriber: MAVLink debug value (from MAVROS) - only if not dummy mode
        if not self.use_dummy:
            self.mavlink_sub = self.create_subscription(
                DebugValue,
                '/mavros/debug_value/recv',
                self.mavlink_callback,
                10
            )
        
        # Publishers: Navigation target and trigger
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
        
        # Timer for buffer timeout check (only when not in dummy mode)
        if not self.use_dummy:
            self.timeout_timer = self.create_timer(1.0, self.check_buffer_timeout)
        
        # Timer for dummy geotag publishing
        if self.use_dummy:
            self.dummy_timer = self.create_timer(self.dummy_interval, self.publish_dummy_geotag)
            self.get_logger().info('=== DUMMY GEOTAG MODE ENABLED ===')
            self.get_logger().info(f'  Target: lat={self.dummy_lat:.6f}, lon={self.dummy_lon:.6f}, alt={self.dummy_alt:.1f}m')
            self.get_logger().info(f'  Interval: {self.dummy_interval}s')
        else:
            self.get_logger().info('Telemetry RX Node initialized')
            self.get_logger().info('  Listening: /mavros/debug_value/recv')
        
        self.get_logger().info('  Publishing: /drone2/target_position')
        self.get_logger().info(f'  Override altitude: {self.override_alt} ({self.target_alt}m)')
    
    def publish_dummy_geotag(self):
        """Publish dummy geotag for testing purposes."""
        if self.dummy_sent:
            return  # Only send once (can be modified for continuous testing)
        
        self.get_logger().info('Publishing DUMMY geotag for testing...')
        
        lat = self.dummy_lat
        lon = self.dummy_lon
        alt = self.dummy_alt
        
        # Validate if configured
        if self.validate_coords and not self._validate_coordinates(lat, lon):
            self.get_logger().error('Dummy geotag rejected: invalid coordinates')
            return
        
        # Dispatch to navigation
        self._dispatch_target(lat, lon, alt)
        self.dummy_sent = True
        self.get_logger().info('Dummy geotag dispatched - waiting for drone response')
    
    def mavlink_callback(self, msg: DebugValue):
        """Buffer incoming MAVLink messages and reconstruct geotag."""
        # Only process NAMED_VALUE_FLOAT type messages
        if msg.type != DebugValue.TYPE_NAMED_VALUE_FLOAT:
            return
        
        name = msg.name.strip()
        
        # Only process disease geotag messages
        if name not in ('d_lat', 'd_lon', 'd_alt'):
            return
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        msg_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        
        # Check for new message group (different timestamp = new geotag)
        if self.buffer_stamp is not None and abs(msg_stamp - self.buffer_stamp) > 0.1:
            # New message group, clear old buffer
            self._clear_buffer()
        
        self.buffer_stamp = msg_stamp
        self.last_buffer_time = current_time
        
        # Store value in buffer
        if name == 'd_lat':
            self.buffer_lat = msg.value_float
        elif name == 'd_lon':
            self.buffer_lon = msg.value_float
        elif name == 'd_alt':
            self.buffer_alt = msg.value_float
        
        # Check if complete
        if self._buffer_complete():
            self._process_geotag()
            self._clear_buffer()
    
    def _buffer_complete(self) -> bool:
        """Check if all components are buffered."""
        return (
            self.buffer_lat is not None and
            self.buffer_lon is not None and
            self.buffer_alt is not None
        )
    
    def _process_geotag(self):
        """Validate and dispatch reconstructed geotag."""
        self.rx_count += 1
        
        lat = self.buffer_lat
        lon = self.buffer_lon
        alt = self.buffer_alt
        
        self.get_logger().info(
            f'RX #{self.rx_count}: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}'
        )
        
        # Validate coordinates
        if self.validate_coords and not self._validate_coordinates(lat, lon):
            self.rejected_count += 1
            self.get_logger().error('Target rejected: invalid coordinates')
            return
        
        # Dispatch to navigation
        self._dispatch_target(lat, lon, alt)
    
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
    
    def _dispatch_target(self, lat: float, lon: float, alt: float):
        """Dispatch target to navigation system."""
        # Create NavSatFix message
        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.latitude = lat
        msg.longitude = lon
        
        # Override altitude if configured
        if self.override_alt:
            msg.altitude = self.target_alt
        else:
            msg.altitude = alt
        
        msg.status.status = 0  # STATUS_FIX
        msg.status.service = 1  # SERVICE_GPS
        
        self.position_pub.publish(msg)
        
        # Send trigger
        trigger_msg = Bool()
        trigger_msg.data = True
        self.trigger_pub.publish(trigger_msg)
        
        self.dispatched_count += 1
        
        self.get_logger().info(
            f'Dispatched #{self.dispatched_count}: lat={msg.latitude:.6f}, '
            f'lon={msg.longitude:.6f}, alt={msg.altitude:.1f}m'
        )
    
    def _clear_buffer(self):
        """Clear message buffer."""
        self.buffer_lat = None
        self.buffer_lon = None
        self.buffer_alt = None
        self.buffer_stamp = None
    
    def check_buffer_timeout(self):
        """Clear buffer if messages stall."""
        if self.last_buffer_time is None:
            return
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.last_buffer_time
        
        if elapsed > self.buffer_timeout and not self._buffer_complete():
            self.get_logger().warn(
                f'Buffer timeout ({elapsed:.1f}s) - clearing incomplete geotag'
            )
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
            f'Statistics: received={node.rx_count}, '
            f'dispatched={node.dispatched_count}, '
            f'rejected={node.rejected_count}'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
