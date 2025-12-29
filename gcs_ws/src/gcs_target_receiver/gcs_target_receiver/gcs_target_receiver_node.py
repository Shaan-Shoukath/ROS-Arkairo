#!/usr/bin/env python3
# Copyright 2024 Maintainer
# SPDX-License-Identifier: Apache-2.0

"""
GCS Target Receiver Node

Receives target reports from Drone-1, validates them, filters duplicates,
and publishes validated targets for mission dispatch.

Subscribers:
    /gcs/target_report (geographic_msgs/GeoPointStamped): From Drone-1 uplink

Publishers:
    /gcs/validated_target (geographic_msgs/GeoPointStamped): Validated targets
    /gcs/new_target (std_msgs/Bool): Trigger for new target received
"""

import math
import json
from datetime import datetime
from typing import List, Tuple
from dataclasses import dataclass, asdict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, Header
from geographic_msgs.msg import GeoPointStamped, GeoPoint


@dataclass
class Target:
    """Validated target entry."""
    target_id: int
    latitude: float
    longitude: float
    altitude: float
    received_time: str
    source: str = "drone1"


class GcsTargetReceiverNode(Node):
    """ROS2 node for receiving and validating target reports."""
    
    EARTH_RADIUS = 6371000  # meters
    
    def __init__(self):
        super().__init__('gcs_target_receiver_node')
        
        # Declare parameters
        self.declare_parameter('duplicate_distance_threshold_m', 3.0)
        self.declare_parameter('duplicate_time_threshold_sec', 60.0)
        self.declare_parameter('min_latitude', -90.0)
        self.declare_parameter('max_latitude', 90.0)
        self.declare_parameter('min_longitude', -180.0)
        self.declare_parameter('max_longitude', 180.0)
        self.declare_parameter('enable_geofence', False)
        self.declare_parameter('geofence_center_lat', 0.0)
        self.declare_parameter('geofence_center_lon', 0.0)
        self.declare_parameter('geofence_radius_m', 10000.0)
        self.declare_parameter('log_all_targets', True)
        self.declare_parameter('save_targets_to_file', False)
        self.declare_parameter('target_log_path', '/tmp/gcs_targets.json')
        
        # Get parameters
        self.dup_distance_thresh = self.get_parameter('duplicate_distance_threshold_m').value
        self.dup_time_thresh = self.get_parameter('duplicate_time_threshold_sec').value
        self.min_lat = self.get_parameter('min_latitude').value
        self.max_lat = self.get_parameter('max_latitude').value
        self.min_lon = self.get_parameter('min_longitude').value
        self.max_lon = self.get_parameter('max_longitude').value
        self.enable_geofence = self.get_parameter('enable_geofence').value
        self.geofence_lat = self.get_parameter('geofence_center_lat').value
        self.geofence_lon = self.get_parameter('geofence_center_lon').value
        self.geofence_radius = self.get_parameter('geofence_radius_m').value
        self.log_all = self.get_parameter('log_all_targets').value
        self.save_to_file = self.get_parameter('save_targets_to_file').value
        self.log_path = self.get_parameter('target_log_path').value
        
        # Target tracking
        self.validated_targets: List[Target] = []
        self.next_target_id = 1
        self.recent_positions: List[Tuple[float, float, float]] = []  # lat, lon, time
        
        # Statistics
        self.total_received = 0
        self.total_validated = 0
        self.total_rejected = 0
        
        # QoS for reliable message delivery
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Subscriber
        self.target_sub = self.create_subscription(
            GeoPointStamped,
            '/gcs/target_report',
            self.target_callback,
            reliable_qos
        )
        
        # Publishers
        self.validated_pub = self.create_publisher(
            GeoPointStamped,
            '/gcs/validated_target',
            reliable_qos
        )
        
        self.new_target_pub = self.create_publisher(
            Bool,
            '/gcs/new_target',
            10
        )
        
        self.get_logger().info('GCS Target Receiver Node initialized')
        self.get_logger().info(f'  Duplicate threshold: {self.dup_distance_thresh}m')
        self.get_logger().info(f'  Geofence enabled: {self.enable_geofence}')
    
    def target_callback(self, msg: GeoPointStamped):
        """Process incoming target report."""
        self.total_received += 1
        
        lat = msg.position.latitude
        lon = msg.position.longitude
        alt = msg.position.altitude
        
        if self.log_all:
            self.get_logger().info(
                f'Received target: lat={lat:.6f}, lon={lon:.6f}'
            )
        
        # Validate coordinates
        if not self._validate_coordinates(lat, lon):
            self.total_rejected += 1
            self.get_logger().warn(f'Target rejected: invalid coordinates')
            return
        
        # Check geofence
        if self.enable_geofence and not self._check_geofence(lat, lon):
            self.total_rejected += 1
            self.get_logger().warn(f'Target rejected: outside geofence')
            return
        
        # Check for duplicate
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self._is_duplicate(lat, lon, current_time):
            self.get_logger().debug('Target rejected: duplicate')
            return
        
        # Target is valid - add to tracking
        self.recent_positions.append((lat, lon, current_time))
        self._cleanup_old_positions(current_time)
        
        target = Target(
            target_id=self.next_target_id,
            latitude=lat,
            longitude=lon,
            altitude=alt,
            received_time=datetime.now().isoformat()
        )
        self.validated_targets.append(target)
        self.next_target_id += 1
        self.total_validated += 1
        
        # Publish validated target
        validated_msg = GeoPointStamped()
        validated_msg.header = Header()
        validated_msg.header.stamp = self.get_clock().now().to_msg()
        validated_msg.header.frame_id = 'map'
        validated_msg.position = GeoPoint()
        validated_msg.position.latitude = lat
        validated_msg.position.longitude = lon
        validated_msg.position.altitude = alt
        
        self.validated_pub.publish(validated_msg)
        
        # Publish new target trigger
        trigger_msg = Bool()
        trigger_msg.data = True
        self.new_target_pub.publish(trigger_msg)
        
        self.get_logger().info(
            f'Validated target #{target.target_id}: '
            f'lat={lat:.6f}, lon={lon:.6f}'
        )
        
        # Save to file if enabled
        if self.save_to_file:
            self._save_targets()
    
    def _validate_coordinates(self, lat: float, lon: float) -> bool:
        """Validate GPS coordinates are within bounds."""
        if lat < self.min_lat or lat > self.max_lat:
            return False
        if lon < self.min_lon or lon > self.max_lon:
            return False
        
        # Reject null island (0,0) as likely invalid
        if lat == 0.0 and lon == 0.0:
            return False
        
        return True
    
    def _check_geofence(self, lat: float, lon: float) -> bool:
        """Check if coordinates are within geofence."""
        distance = self._haversine_distance(
            lat, lon,
            self.geofence_lat, self.geofence_lon
        )
        return distance <= self.geofence_radius
    
    def _is_duplicate(self, lat: float, lon: float, current_time: float) -> bool:
        """Check if this is a duplicate of recent target."""
        for prev_lat, prev_lon, prev_time in self.recent_positions:
            # Check time window
            if current_time - prev_time > self.dup_time_thresh:
                continue
            
            # Check distance
            distance = self._haversine_distance(lat, lon, prev_lat, prev_lon)
            if distance < self.dup_distance_thresh:
                return True
        
        return False
    
    def _cleanup_old_positions(self, current_time: float):
        """Remove positions outside time window."""
        self.recent_positions = [
            (lat, lon, t) for lat, lon, t in self.recent_positions
            if current_time - t <= self.dup_time_thresh
        ]
    
    def _haversine_distance(
        self,
        lat1: float, lon1: float,
        lat2: float, lon2: float
    ) -> float:
        """Calculate distance between two GPS points in meters."""
        lat1, lon1 = math.radians(lat1), math.radians(lon1)
        lat2, lon2 = math.radians(lat2), math.radians(lon2)
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = math.sin(dlat/2)**2 + \
            math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        
        return self.EARTH_RADIUS * c
    
    def _save_targets(self):
        """Save validated targets to JSON file."""
        try:
            data = {
                'targets': [asdict(t) for t in self.validated_targets],
                'statistics': {
                    'total_received': self.total_received,
                    'total_validated': self.total_validated,
                    'total_rejected': self.total_rejected
                }
            }
            with open(self.log_path, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            self.get_logger().error(f'Failed to save targets: {e}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = GcsTargetReceiverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Statistics: received={node.total_received}, '
            f'validated={node.total_validated}, '
            f'rejected={node.total_rejected}'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
