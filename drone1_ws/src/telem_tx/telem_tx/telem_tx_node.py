#!/usr/bin/env python3
# Copyright 2024 Shaan Shoukath
# SPDX-License-Identifier: Apache-2.0

"""
Telemetry TX Node

Transmits disease geotags over MAVLink telemetry to GCS/Drone 2.

Supports two encoding modes:
    1. STATUSTEXT (default) - Single message with "GEOTAG:lat,lon,alt" format
    2. NAMED_VALUE_FLOAT - Three separate messages (d_lat, d_lon, d_alt)

Subscribers:
    /drone1/disease_geotag (geographic_msgs/GeoPointStamped): Disease detections

Publishers:
    /mavros/statustext/send (mavros_msgs/StatusText): STATUSTEXT mode
    /mavros/debug_value/send (mavros_msgs/DebugValue): NAMED_VALUE_FLOAT mode
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import DebugValue, StatusText


class TelemTxNode(Node):
    """ROS2 node for transmitting disease geotags over telemetry."""
    
    GEOTAG_PREFIX = "GEOTAG:"
    
    def __init__(self):
        super().__init__('telem_tx_node')
        
        # Parameters
        self.declare_parameter('use_statustext', True)  # True = STATUSTEXT, False = NAMED_VALUE_FLOAT
        self.use_statustext = self.get_parameter('use_statustext').value
        
        # Statistics
        self.tx_count = 0
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscriber: disease geotags from detection node
        self.geotag_sub = self.create_subscription(
            GeoPointStamped,
            '/drone1/disease_geotag',
            self.geotag_callback,
            sensor_qos
        )
        
        # Publishers
        if self.use_statustext:
            self.statustext_pub = self.create_publisher(
                StatusText,
                '/mavros/statustext/send',
                10
            )
            self.get_logger().info('Mode: STATUSTEXT (single message)')
        else:
            self.debug_pub = self.create_publisher(
                DebugValue,
                '/mavros/debug_value/send',
                10
            )
            self.get_logger().info('Mode: NAMED_VALUE_FLOAT (3 messages)')
        
        self.get_logger().info('Telemetry TX Node initialized')
        self.get_logger().info('  Listening: /drone1/disease_geotag')
    
    def geotag_callback(self, msg: GeoPointStamped):
        """Convert geotag to MAVLink message and transmit."""
        lat = msg.position.latitude
        lon = msg.position.longitude
        alt = msg.position.altitude
        
        if self.use_statustext:
            self._send_statustext(lat, lon, alt)
        else:
            self._send_named_values(lat, lon, alt)
        
        self.tx_count += 1
        self.get_logger().info(
            f'TX #{self.tx_count}: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}'
        )
    
    def _send_statustext(self, lat: float, lon: float, alt: float):
        """Send geotag as single STATUSTEXT message."""
        # Format: GEOTAG:lat,lon,alt
        text = f"{self.GEOTAG_PREFIX}{lat:.6f},{lon:.6f},{alt:.1f}"
        
        msg = StatusText()
        msg.severity = 6  # INFO level
        msg.text = text[:50]  # MAVLink limit
        
        self.statustext_pub.publish(msg)
        self.get_logger().debug(f'STATUSTEXT: {text}')
    
    def _send_named_values(self, lat: float, lon: float, alt: float):
        """Send geotag as three NAMED_VALUE_FLOAT messages."""
        now = self.get_clock().now().to_msg()
        
        # Send latitude
        lat_msg = DebugValue()
        lat_msg.header.stamp = now
        lat_msg.header.frame_id = 'map'
        lat_msg.index = -1
        lat_msg.array_id = -1
        lat_msg.name = 'd_lat'
        lat_msg.value_float = float(lat)
        lat_msg.type = DebugValue.TYPE_NAMED_VALUE_FLOAT
        self.debug_pub.publish(lat_msg)
        
        # Send longitude
        lon_msg = DebugValue()
        lon_msg.header.stamp = now
        lon_msg.header.frame_id = 'map'
        lon_msg.index = -1
        lon_msg.array_id = -1
        lon_msg.name = 'd_lon'
        lon_msg.value_float = float(lon)
        lon_msg.type = DebugValue.TYPE_NAMED_VALUE_FLOAT
        self.debug_pub.publish(lon_msg)
        
        # Send altitude
        alt_msg = DebugValue()
        alt_msg.header.stamp = now
        alt_msg.header.frame_id = 'map'
        alt_msg.index = -1
        alt_msg.array_id = -1
        alt_msg.name = 'd_alt'
        alt_msg.value_float = float(alt)
        alt_msg.type = DebugValue.TYPE_NAMED_VALUE_FLOAT
        self.debug_pub.publish(alt_msg)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = TelemTxNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'Total transmitted: {node.tx_count}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
