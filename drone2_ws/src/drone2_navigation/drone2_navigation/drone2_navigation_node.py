#!/usr/bin/env python3
# Copyright 2024 Maintainer
# SPDX-License-Identifier: Apache-2.0

"""
Drone-2 Navigation Node

Navigates Drone-2 to target location using global position setpoints.
Reports arrival status for mission sequencing.

Subscribers:
    /drone2/target_position (sensor_msgs/NavSatFix): Target from downlink
    /mavros/global_position/global (sensor_msgs/NavSatFix): Current GPS
    /mavros/state (mavros_msgs/State): Flight controller state

Publishers:
    /mavros/setpoint_position/global (mavros_msgs/GlobalPositionTarget): Target
    /drone2/arrival_status (std_msgs/Bool): Arrival at target
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, Header
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, GlobalPositionTarget


class Drone2NavigationNode(Node):
    """ROS2 node for Drone-2 target navigation."""
    
    EARTH_RADIUS = 6371000  # meters
    
    def __init__(self):
        super().__init__('drone2_navigation_node')
        
        # Declare parameters
        self.declare_parameter('arrival_radius_m', 2.0)
        self.declare_parameter('approach_speed', 3.0)
        self.declare_parameter('approach_altitude_m', 10.0)
        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('navigation_timeout_sec', 120.0)
        
        # Get parameters
        self.arrival_radius = self.get_parameter('arrival_radius_m').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.approach_alt = self.get_parameter('approach_altitude_m').value
        update_rate = self.get_parameter('update_rate_hz').value
        self.nav_timeout = self.get_parameter('navigation_timeout_sec').value
        
        # State
        self.target_position: Optional[NavSatFix] = None
        self.current_position: Optional[NavSatFix] = None
        self.fc_state: Optional[State] = None
        self.navigating = False
        self.arrived = False
        self.nav_start_time = None
        
        # QoS profiles
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
        self.target_sub = self.create_subscription(
            NavSatFix,
            '/drone2/target_position',
            self.target_callback,
            reliable_qos
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            sensor_qos
        )
        
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            sensor_qos
        )
        
        # Publishers
        self.setpoint_pub = self.create_publisher(
            GlobalPositionTarget,
            '/mavros/setpoint_position/global',
            10
        )
        
        self.arrival_pub = self.create_publisher(
            Bool,
            '/drone2/arrival_status',
            reliable_qos
        )
        
        # Navigation timer
        self.nav_timer = self.create_timer(
            1.0 / update_rate,
            self.navigation_loop
        )
        
        self.get_logger().info('Drone-2 Navigation Node initialized')
        self.get_logger().info(f'  Arrival radius: {self.arrival_radius}m')
        self.get_logger().info(f'  Approach altitude: {self.approach_alt}m')
    
    def target_callback(self, msg: NavSatFix):
        """Receive new target position."""
        self.target_position = msg
        self.navigating = True
        self.arrived = False
        self.nav_start_time = self.get_clock().now()
        
        self.get_logger().info(
            f'New target: lat={msg.latitude:.6f}, '
            f'lon={msg.longitude:.6f}, alt={msg.altitude:.1f}m'
        )
    
    def gps_callback(self, msg: NavSatFix):
        """Update current GPS position."""
        self.current_position = msg
    
    def state_callback(self, msg: State):
        """Update flight controller state."""
        self.fc_state = msg
    
    def navigation_loop(self):
        """Main navigation control loop."""
        if not self.navigating:
            return
        
        if self.target_position is None:
            return
        
        if self.current_position is None:
            self.get_logger().warn('No GPS position', throttle_duration_sec=5.0)
            return
        
        # Check timeout
        if self._check_timeout():
            self.get_logger().error('Navigation timeout')
            self.navigating = False
            return
        
        # Calculate distance to target
        distance = self._distance_to_target()
        
        # Check arrival
        if distance < self.arrival_radius and not self.arrived:
            self.arrived = True
            self.navigating = False
            
            # Publish arrival
            msg = Bool()
            msg.data = True
            self.arrival_pub.publish(msg)
            
            self.get_logger().info(
                f'Arrived at target (distance: {distance:.2f}m)'
            )
            return
        
        # Publish setpoint
        self._publish_setpoint()
    
    def _distance_to_target(self) -> float:
        """Calculate distance to target in meters."""
        lat1 = math.radians(self.current_position.latitude)
        lon1 = math.radians(self.current_position.longitude)
        lat2 = math.radians(self.target_position.latitude)
        lon2 = math.radians(self.target_position.longitude)
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = math.sin(dlat/2)**2 + \
            math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        
        return self.EARTH_RADIUS * c
    
    def _check_timeout(self) -> bool:
        """Check if navigation has timed out."""
        if self.nav_start_time is None:
            return False
        
        elapsed = (self.get_clock().now() - self.nav_start_time).nanoseconds / 1e9
        return elapsed > self.nav_timeout
    
    def _publish_setpoint(self):
        """Publish global position setpoint."""
        msg = GlobalPositionTarget()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        msg.type_mask = (
            GlobalPositionTarget.IGNORE_VX |
            GlobalPositionTarget.IGNORE_VY |
            GlobalPositionTarget.IGNORE_VZ |
            GlobalPositionTarget.IGNORE_AFX |
            GlobalPositionTarget.IGNORE_AFY |
            GlobalPositionTarget.IGNORE_AFZ |
            GlobalPositionTarget.IGNORE_YAW_RATE
        )
        
        msg.latitude = self.target_position.latitude
        msg.longitude = self.target_position.longitude
        msg.altitude = self.target_position.altitude
        
        # Calculate yaw towards target
        msg.yaw = self._calculate_bearing()
        
        self.setpoint_pub.publish(msg)
    
    def _calculate_bearing(self) -> float:
        """Calculate bearing to target."""
        lat1 = math.radians(self.current_position.latitude)
        lon1 = math.radians(self.current_position.longitude)
        lat2 = math.radians(self.target_position.latitude)
        lon2 = math.radians(self.target_position.longitude)
        
        dlon = lon2 - lon1
        
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - \
            math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        
        return math.atan2(x, y)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = Drone2NavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
