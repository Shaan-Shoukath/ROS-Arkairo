#!/usr/bin/env python3
# Copyright 2024 Maintainer
# SPDX-License-Identifier: Apache-2.0

"""
D1 to GCS Uplink Node

Forwards validated disease geotags from Drone-1 to Ground Control Station
over telemetry link with reliable QoS.

Subscribers:
    /drone1/disease_geotag (geographic_msgs/GeoPointStamped): Local detections

Publishers:
    /gcs/target_report (geographic_msgs/GeoPointStamped): For GCS consumption
"""

from typing import Deque
from collections import deque
from dataclasses import dataclass
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Header, String
from geographic_msgs.msg import GeoPointStamped


@dataclass
class QueuedTarget:
    """Target awaiting transmission."""
    geotag: GeoPointStamped
    timestamp: datetime
    retries: int = 0


class D1ToGcsUplinkNode(Node):
    """ROS2 node for forwarding disease geotags to GCS."""
    
    def __init__(self):
        super().__init__('d1_to_gcs_uplink_node')
        
        # Declare parameters
        self.declare_parameter('max_queue_size', 100)
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('retry_interval_sec', 1.0)
        self.declare_parameter('min_publish_interval_sec', 0.5)
        self.declare_parameter('enable_heartbeat', True)
        self.declare_parameter('heartbeat_interval_sec', 5.0)
        self.declare_parameter('log_all_targets', True)
        
        # Get parameters
        self.max_queue_size = self.get_parameter('max_queue_size').value
        self.max_retries = self.get_parameter('max_retries').value
        self.retry_interval = self.get_parameter('retry_interval_sec').value
        self.min_publish_interval = self.get_parameter('min_publish_interval_sec').value
        self.enable_heartbeat = self.get_parameter('enable_heartbeat').value
        self.heartbeat_interval = self.get_parameter('heartbeat_interval_sec').value
        self.log_all = self.get_parameter('log_all_targets').value
        
        # Transmission queue
        self.tx_queue: Deque[QueuedTarget] = deque(maxlen=self.max_queue_size)
        self.last_publish_time = self.get_clock().now()
        
        # Statistics
        self.total_received = 0
        self.total_forwarded = 0
        self.total_failed = 0
        
        # QoS for reliable telemetry
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Subscriber - from local detection
        self.geotag_sub = self.create_subscription(
            GeoPointStamped,
            '/drone1/disease_geotag',
            self.geotag_callback,
            reliable_qos
        )
        
        # Publisher - to GCS
        self.gcs_pub = self.create_publisher(
            GeoPointStamped,
            '/gcs/target_report',
            reliable_qos
        )
        
        # Heartbeat publisher (for link monitoring)
        if self.enable_heartbeat:
            self.heartbeat_pub = self.create_publisher(
                String,
                '/drone1/uplink_heartbeat',
                10
            )
            self.heartbeat_timer = self.create_timer(
                self.heartbeat_interval,
                self.publish_heartbeat
            )
        
        # Queue processing timer
        self.process_timer = self.create_timer(
            self.min_publish_interval,
            self.process_queue
        )
        
        self.get_logger().info('D1 to GCS Uplink Node initialized')
        self.get_logger().info(f'  Max queue size: {self.max_queue_size}')
        self.get_logger().info(f'  Heartbeat enabled: {self.enable_heartbeat}')
    
    def geotag_callback(self, msg: GeoPointStamped):
        """Receive geotag and queue for transmission."""
        self.total_received += 1
        
        # Create queued target
        target = QueuedTarget(
            geotag=msg,
            timestamp=datetime.now()
        )
        
        # Add to queue
        if len(self.tx_queue) >= self.max_queue_size:
            self.get_logger().warn('Queue full, dropping oldest target')
            self.tx_queue.popleft()
        
        self.tx_queue.append(target)
        
        if self.log_all:
            self.get_logger().info(
                f'Queued target: lat={msg.position.latitude:.6f}, '
                f'lon={msg.position.longitude:.6f} '
                f'(queue size: {len(self.tx_queue)})'
            )
    
    def process_queue(self):
        """Process transmission queue."""
        if not self.tx_queue:
            return
        
        # Check rate limiting
        now = self.get_clock().now()
        elapsed = (now - self.last_publish_time).nanoseconds / 1e9
        
        if elapsed < self.min_publish_interval:
            return
        
        # Get next target
        target = self.tx_queue.popleft()
        
        # Forward to GCS
        try:
            # Update timestamp
            target.geotag.header.stamp = now.to_msg()
            
            self.gcs_pub.publish(target.geotag)
            self.last_publish_time = now
            self.total_forwarded += 1
            
            self.get_logger().info(
                f'Forwarded target to GCS: '
                f'lat={target.geotag.position.latitude:.6f}, '
                f'lon={target.geotag.position.longitude:.6f}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Failed to forward target: {e}')
            target.retries += 1
            
            if target.retries < self.max_retries:
                # Re-queue for retry
                self.tx_queue.appendleft(target)
            else:
                self.total_failed += 1
                self.get_logger().error(
                    f'Target failed after {self.max_retries} retries, dropping'
                )
    
    def publish_heartbeat(self):
        """Publish heartbeat message."""
        msg = String()
        msg.data = (
            f'D1_UPLINK|'
            f'rx={self.total_received}|'
            f'tx={self.total_forwarded}|'
            f'fail={self.total_failed}|'
            f'queue={len(self.tx_queue)}'
        )
        self.heartbeat_pub.publish(msg)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = D1ToGcsUplinkNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Log final statistics
        node.get_logger().info(
            f'Uplink statistics: '
            f'received={node.total_received}, '
            f'forwarded={node.total_forwarded}, '
            f'failed={node.total_failed}'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
