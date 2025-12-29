#!/usr/bin/env python3
# Copyright 2024 Maintainer
# SPDX-License-Identifier: Apache-2.0

"""
Centering Controller Node

Fine-tunes drone position to center spray nozzle over detected target
using visual servoing with velocity commands.

Subscribers:
    /drone2/local_bbox (vision_msgs/BoundingBox2D): Detection bounding box
    /drone2/local_detection_status (std_msgs/Bool): Detection confirmed

Publishers:
    /mavros/setpoint_velocity/cmd_vel (geometry_msgs/TwistStamped): Velocity
    /drone2/spray_ready (std_msgs/Bool): Spray alignment confirmed
"""

from typing import Optional
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, Header
from geometry_msgs.msg import TwistStamped
from vision_msgs.msg import BoundingBox2D


@dataclass
class PIDState:
    """PID controller state."""
    integral_x: float = 0.0
    integral_y: float = 0.0
    prev_error_x: float = 0.0
    prev_error_y: float = 0.0


class CenteringControllerNode(Node):
    """ROS2 node for visual servoing centering control."""
    
    def __init__(self):
        super().__init__('centering_controller_node')
        
        # Declare parameters
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)
        self.declare_parameter('centered_threshold_pixels', 30)
        self.declare_parameter('max_velocity_mps', 0.5)
        self.declare_parameter('kp', 0.002)
        self.declare_parameter('ki', 0.0001)
        self.declare_parameter('kd', 0.001)
        self.declare_parameter('centering_timeout_sec', 30.0)
        self.declare_parameter('control_rate_hz', 20.0)
        
        # Get parameters
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.centered_thresh = self.get_parameter('centered_threshold_pixels').value
        self.max_vel = self.get_parameter('max_velocity_mps').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.timeout = self.get_parameter('centering_timeout_sec').value
        control_rate = self.get_parameter('control_rate_hz').value
        
        # Image center
        self.center_x = self.image_width / 2
        self.center_y = self.image_height / 2
        
        # State
        self.centering_active = False
        self.centering_start_time = None
        self.last_bbox: Optional[BoundingBox2D] = None
        self.pid = PIDState()
        self.dt = 1.0 / control_rate
        
        # QoS
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Subscribers
        self.bbox_sub = self.create_subscription(
            BoundingBox2D, '/drone2/local_bbox',
            self.bbox_callback, 10
        )
        
        self.status_sub = self.create_subscription(
            Bool, '/drone2/local_detection_status',
            self.status_callback, reliable_qos
        )
        
        # Publishers
        self.vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10
        )
        
        self.ready_pub = self.create_publisher(
            Bool, '/drone2/spray_ready', reliable_qos
        )
        
        # Control timer
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info('Centering Controller Node initialized')
        self.get_logger().info(f'  Centered threshold: {self.centered_thresh}px')
    
    def status_callback(self, msg: Bool):
        """Start centering when detection confirmed."""
        if msg.data and not self.centering_active:
            self.centering_active = True
            self.centering_start_time = self.get_clock().now()
            self.pid = PIDState()  # Reset PID
            self.get_logger().info('Starting centering control')
    
    def bbox_callback(self, msg: BoundingBox2D):
        """Update target bounding box."""
        self.last_bbox = msg
    
    def control_loop(self):
        """Main control loop."""
        if not self.centering_active:
            return
        
        # Check timeout
        elapsed = (self.get_clock().now() - self.centering_start_time).nanoseconds / 1e9
        if elapsed > self.timeout:
            self.get_logger().warn('Centering timeout')
            self.centering_active = False
            self._publish_ready(False)
            return
        
        if self.last_bbox is None:
            return
        
        # Calculate error from image center
        error_x = self.last_bbox.center.position.x - self.center_x
        error_y = self.last_bbox.center.position.y - self.center_y
        
        # Check if centered
        if abs(error_x) < self.centered_thresh and abs(error_y) < self.centered_thresh:
            self.get_logger().info(
                f'Target centered (error: {error_x:.1f}, {error_y:.1f})'
            )
            self.centering_active = False
            self._stop_motion()
            self._publish_ready(True)
            return
        
        # PID control
        # Update integrals
        self.pid.integral_x += error_x * self.dt
        self.pid.integral_y += error_y * self.dt
        
        # Calculate derivatives
        deriv_x = (error_x - self.pid.prev_error_x) / self.dt
        deriv_y = (error_y - self.pid.prev_error_y) / self.dt
        
        # Calculate velocities (note: image X maps to body Y, image Y to body X)
        vel_y = -(self.kp * error_x + self.ki * self.pid.integral_x + self.kd * deriv_x)
        vel_x = -(self.kp * error_y + self.ki * self.pid.integral_y + self.kd * deriv_y)
        
        # Clamp velocities
        vel_x = max(-self.max_vel, min(self.max_vel, vel_x))
        vel_y = max(-self.max_vel, min(self.max_vel, vel_y))
        
        # Update previous errors
        self.pid.prev_error_x = error_x
        self.pid.prev_error_y = error_y
        
        # Publish velocity command
        self._publish_velocity(vel_x, vel_y)
    
    def _publish_velocity(self, vx: float, vy: float):
        """Publish velocity command."""
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = 0.0
        
        self.vel_pub.publish(msg)
    
    def _stop_motion(self):
        """Stop all motion."""
        self._publish_velocity(0.0, 0.0)
    
    def _publish_ready(self, ready: bool):
        """Publish spray ready status."""
        msg = Bool()
        msg.data = ready
        self.ready_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CenteringControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
