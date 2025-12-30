#!/usr/bin/env python3
# Copyright 2024 Maintainer
# SPDX-License-Identifier: Apache-2.0

"""
Telemetry TX Node

Transmits disease geotags over MAVLink telemetry to Drone 2.

Subscribers:
    /drone1/disease_geotag (geographic_msgs/GeoPointStamped): Disease detections

Publishers:
    /mavros/named_value_float/send (mavros_msgs/NamedValueFloat): MAVLink transport

Encoding:
    d_lat -> latitude
    d_lon -> longitude
    d_alt -> altitude
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import NamedValueFloat


class TelemTxNode(Node):
    """ROS2 node for transmitting disease geotags over telemetry."""
    
    def __init__(self):
        super().__init__('telem_tx_node')
        
        # Statistics
        self.tx_count = 0
        
        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Subscriber: disease geotags from detection node
        self.geotag_sub = self.create_subscription(
            GeoPointStamped,
            '/drone1/disease_geotag',
            self.geotag_callback,
            reliable_qos
        )
        
        # Publisher: MAVLink named value float (to MAVROS)
        self.mavlink_pub = self.create_publisher(
            NamedValueFloat,
            '/mavros/named_value_float/send',
            10
        )
        
        self.get_logger().info('Telemetry TX Node initialized')
        self.get_logger().info('  Listening: /drone1/disease_geotag')
        self.get_logger().info('  Publishing: /mavros/named_value_float/send')
    
    def geotag_callback(self, msg: GeoPointStamped):
        """Convert geotag to MAVLink messages and transmit."""
        lat = msg.position.latitude
        lon = msg.position.longitude
        alt = msg.position.altitude
        
        # Get timestamp for synchronization
        time_boot_ms = int(self.get_clock().now().nanoseconds / 1_000_000) & 0xFFFFFFFF
        
        # Send latitude
        lat_msg = NamedValueFloat()
        lat_msg.time_boot_ms = time_boot_ms
        lat_msg.name = 'd_lat'
        lat_msg.value = float(lat)
        self.mavlink_pub.publish(lat_msg)
        
        # Send longitude
        lon_msg = NamedValueFloat()
        lon_msg.time_boot_ms = time_boot_ms
        lon_msg.name = 'd_lon'
        lon_msg.value = float(lon)
        self.mavlink_pub.publish(lon_msg)
        
        # Send altitude
        alt_msg = NamedValueFloat()
        alt_msg.time_boot_ms = time_boot_ms
        alt_msg.name = 'd_alt'
        alt_msg.value = float(alt)
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
