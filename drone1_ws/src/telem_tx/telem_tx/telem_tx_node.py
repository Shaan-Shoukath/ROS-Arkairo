#!/usr/bin/env python3
# Copyright 2024 Shaan Shoukath
# SPDX-License-Identifier: Apache-2.0

"""
Telemetry TX Node

Transmits disease geotags over MAVLink telemetry to Drone 2.

Subscribers:
    /drone1/disease_geotag (geographic_msgs/GeoPointStamped): Disease detections

Publishers:
    /mavros/debug_value/send (mavros_msgs/DebugValue): MAVLink transport

Encoding:
    d_lat -> latitude
    d_lon -> longitude
    d_alt -> altitude
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import DebugValue


class TelemTxNode(Node):
    """ROS2 node for transmitting disease geotags over telemetry."""
    
    def __init__(self):
        super().__init__('telem_tx_node')
        
        # Statistics
        self.tx_count = 0
        
        # QoS profiles - using VOLATILE for compatibility with ros2 topic pub
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
        
        # Publisher: MAVLink debug value (to MAVROS)
        self.mavlink_pub = self.create_publisher(
            DebugValue,
            '/mavros/debug_value/send',
            10
        )
        
        self.get_logger().info('Telemetry TX Node initialized')
        self.get_logger().info('  Listening: /drone1/disease_geotag')
        self.get_logger().info('  Publishing: /mavros/debug_value/send')
    
    def geotag_callback(self, msg: GeoPointStamped):
        """Convert geotag to MAVLink messages and transmit."""
        lat = msg.position.latitude
        lon = msg.position.longitude
        alt = msg.position.altitude
        
        # Get current timestamp
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
        self.mavlink_pub.publish(lat_msg)
        
        # Send longitude
        lon_msg = DebugValue()
        lon_msg.header.stamp = now
        lon_msg.header.frame_id = 'map'
        lon_msg.index = -1
        lon_msg.array_id = -1
        lon_msg.name = 'd_lon'
        lon_msg.value_float = float(lon)
        lon_msg.type = DebugValue.TYPE_NAMED_VALUE_FLOAT
        self.mavlink_pub.publish(lon_msg)
        
        # Send altitude
        alt_msg = DebugValue()
        alt_msg.header.stamp = now
        alt_msg.header.frame_id = 'map'
        alt_msg.index = -1
        alt_msg.array_id = -1
        alt_msg.name = 'd_alt'
        alt_msg.value_float = float(alt)
        alt_msg.type = DebugValue.TYPE_NAMED_VALUE_FLOAT
        self.mavlink_pub.publish(alt_msg)
        
        self.tx_count += 1
        self.get_logger().info(
            f'TX #{self.tx_count}: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}'
        )


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
