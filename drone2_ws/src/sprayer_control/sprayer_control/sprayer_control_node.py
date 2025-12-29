#!/usr/bin/env python3
# Copyright 2024 Maintainer
# SPDX-License-Identifier: Apache-2.0

"""
Sprayer Control Node

Controls spray actuation via PWM output with safety checks.

Subscribers:
    /drone2/spray_ready (std_msgs/Bool): Spray activation trigger
    /mavros/state (mavros_msgs/State): Flight controller state

Publishers:
    /drone2/spray_done (std_msgs/Bool): Spray operation complete
    /drone2/pwm_spray (std_msgs/UInt16): PWM output value
"""

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, UInt16
from mavros_msgs.msg import State


class SprayerControlNode(Node):
    """ROS2 node for spray actuation control."""
    
    def __init__(self):
        super().__init__('sprayer_control_node')
        
        # Declare parameters
        self.declare_parameter('pwm_channel', 9)
        self.declare_parameter('pwm_off', 1000)
        self.declare_parameter('pwm_on', 2000)
        self.declare_parameter('spray_duration_sec', 5.0)
        self.declare_parameter('require_armed', True)
        self.declare_parameter('require_guided', True)
        self.declare_parameter('min_altitude_m', 2.0)
        
        # Get parameters
        self.pwm_channel = self.get_parameter('pwm_channel').value
        self.pwm_off = self.get_parameter('pwm_off').value
        self.pwm_on = self.get_parameter('pwm_on').value
        self.spray_duration = self.get_parameter('spray_duration_sec').value
        self.require_armed = self.get_parameter('require_armed').value
        self.require_guided = self.get_parameter('require_guided').value
        self.min_alt = self.get_parameter('min_altitude_m').value
        
        # State
        self.fc_state: Optional[State] = None
        self.spraying = False
        self.spray_start_time = None
        
        # Statistics
        self.total_sprays = 0
        
        # QoS
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
        self.ready_sub = self.create_subscription(
            Bool, '/drone2/spray_ready',
            self.ready_callback, reliable_qos
        )
        
        self.state_sub = self.create_subscription(
            State, '/mavros/state',
            self.state_callback, sensor_qos
        )
        
        # Publishers
        self.done_pub = self.create_publisher(
            Bool, '/drone2/spray_done', reliable_qos
        )
        
        self.pwm_pub = self.create_publisher(
            UInt16, '/drone2/pwm_spray', 10
        )
        
        # Spray control timer
        self.spray_timer = self.create_timer(0.1, self.spray_loop)
        
        # Ensure sprayer is off at start
        self._set_pwm(self.pwm_off)
        
        self.get_logger().info('Sprayer Control Node initialized')
        self.get_logger().info(f'  Spray duration: {self.spray_duration}s')
        self.get_logger().info(f'  PWM: {self.pwm_off} (off) -> {self.pwm_on} (on)')
    
    def state_callback(self, msg: State):
        """Update flight controller state."""
        self.fc_state = msg
    
    def ready_callback(self, msg: Bool):
        """Handle spray ready trigger."""
        if not msg.data:
            return
        
        if self.spraying:
            self.get_logger().warn('Already spraying')
            return
        
        # Safety checks
        if not self._safety_check():
            self.get_logger().error('Safety check failed - spray aborted')
            self._publish_done(False)
            return
        
        # Start spraying
        self.spraying = True
        self.spray_start_time = self.get_clock().now()
        self._set_pwm(self.pwm_on)
        self.total_sprays += 1
        
        self.get_logger().info('Spraying started')
    
    def spray_loop(self):
        """Monitor spray operation."""
        if not self.spraying:
            return
        
        # Check duration
        elapsed = (self.get_clock().now() - self.spray_start_time).nanoseconds / 1e9
        
        if elapsed >= self.spray_duration:
            self._stop_spray()
            self._publish_done(True)
            self.get_logger().info(f'Spraying complete ({elapsed:.1f}s)')
    
    def _safety_check(self) -> bool:
        """Perform safety checks before spraying."""
        if self.fc_state is None:
            self.get_logger().warn('No FC state available')
            return False
        
        if self.require_armed and not self.fc_state.armed:
            self.get_logger().warn('Vehicle not armed')
            return False
        
        if self.require_guided and self.fc_state.mode != 'GUIDED':
            self.get_logger().warn(f'Not in GUIDED mode: {self.fc_state.mode}')
            return False
        
        return True
    
    def _set_pwm(self, value: int):
        """Set PWM output value."""
        msg = UInt16()
        msg.data = value
        self.pwm_pub.publish(msg)
    
    def _stop_spray(self):
        """Stop spraying."""
        self.spraying = False
        self._set_pwm(self.pwm_off)
    
    def _publish_done(self, success: bool):
        """Publish spray done status."""
        msg = Bool()
        msg.data = success
        self.done_pub.publish(msg)
    
    def destroy_node(self):
        """Ensure sprayer is off on shutdown."""
        self._set_pwm(self.pwm_off)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SprayerControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'Total sprays: {node.total_sprays}')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
