#!/usr/bin/env python3
# Copyright 2024 Shaan Shoukath
# SPDX-License-Identifier: Apache-2.0

"""
Detection and Centering Node (MERGED)

Combined local disease detection and visual servoing for Drone-2.
Optimized workflow: Arrival → Detection → Centering → Spray Ready

Subscribers:
    /drone2/arrival_status (std_msgs/Bool): Trigger to start detection
    /camera/image_raw (sensor_msgs/Image): Camera feed
    /mavros/local_position/pose (geometry_msgs/PoseStamped): Current pose

Publishers:
    /mavros/setpoint_velocity/cmd_vel (geometry_msgs/TwistStamped): Velocity
    /drone2/spray_ready (std_msgs/Bool): Spray alignment confirmed
    /drone2/detection_status (std_msgs/Bool): Detection result (for logging)
"""

import cv2
import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TwistStamped
from vision_msgs.msg import BoundingBox2D
from cv_bridge import CvBridge


class State(Enum):
    """Detection and centering state machine."""
    IDLE = auto()         # Waiting for arrival trigger
    DETECTING = auto()    # Looking for disease
    CENTERING = auto()    # Visual servoing to center
    COMPLETED = auto()    # Ready to spray


@dataclass
class PIDState:
    """PID controller state."""
    integral_x: float = 0.0
    integral_y: float = 0.0
    prev_error_x: float = 0.0
    prev_error_y: float = 0.0


class DetectionCenteringNode(Node):
    """ROS2 node for combined detection and centering."""
    
    def __init__(self):
        super().__init__('detection_centering_node')
        
        # Detection parameters
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('detection_timeout_sec', 30.0)
        self.declare_parameter('max_detection_attempts', 5)
        self.declare_parameter('require_consecutive_detections', 2)
        self.declare_parameter('min_detection_area_px', 500)
        
        # Centering parameters
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)
        self.declare_parameter('centered_threshold_pixels', 30)
        self.declare_parameter('max_velocity_mps', 0.5)
        self.declare_parameter('kp', 0.002)
        self.declare_parameter('ki', 0.0001)
        self.declare_parameter('kd', 0.001)
        self.declare_parameter('centering_timeout_sec', 30.0)
        self.declare_parameter('control_rate_hz', 20.0)
        
        # Detection params
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.detection_timeout = self.get_parameter('detection_timeout_sec').value
        self.max_attempts = self.get_parameter('max_detection_attempts').value
        self.consec_required = self.get_parameter('require_consecutive_detections').value
        self.min_area = self.get_parameter('min_detection_area_px').value
        
        # Centering params
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.centered_thresh = self.get_parameter('centered_threshold_pixels').value
        self.max_vel = self.get_parameter('max_velocity_mps').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.centering_timeout = self.get_parameter('centering_timeout_sec').value
        control_rate = self.get_parameter('control_rate_hz').value
        
        # Image center
        self.center_x = self.image_width / 2
        self.center_y = self.image_height / 2
        
        # State machine
        self.state = State.IDLE
        self.state_start_time = None
        
        # Detection state
        self.detection_attempts = 0
        self.consecutive_detections = 0
        self.current_bbox: Optional[Tuple[int, int, int, int]] = None
        
        # Centering state
        self.pid = PIDState()
        self.dt = 1.0 / control_rate
        
        # CV bridge
        self.bridge = CvBridge()
        
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
        self.arrival_sub = self.create_subscription(
            Bool, '/drone2/arrival_status',
            self.arrival_callback, reliable_qos
        )
        
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw',
            self.image_callback, sensor_qos
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.pose_callback, sensor_qos
        )
        
        # Publishers
        self.vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10
        )
        
        self.ready_pub = self.create_publisher(
            Bool, '/drone2/spray_ready', reliable_qos
        )
        
        self.status_pub = self.create_publisher(
            Bool, '/drone2/detection_status', reliable_qos
        )
        
        # Control timer
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info('Detection & Centering Node initialized (MERGED)')
        self.get_logger().info(f'  Detection timeout: {self.detection_timeout}s')
        self.get_logger().info(f'  Centering threshold: {self.centered_thresh}px')
        self.get_logger().info(f'  Max velocity: {self.max_vel} m/s')
    
    def arrival_callback(self, msg: Bool):
        """Start detection when arrived at target."""
        if msg.data and self.state == State.IDLE:
            self._transition_to(State.DETECTING)
            self.get_logger().info('Arrived - starting detection')
    
    def pose_callback(self, msg: PoseStamped):
        """Update current pose (for future enhancements)."""
        pass
    
    def image_callback(self, msg: Image):
        """Process image for detection and centering."""
        if self.state == State.DETECTING:
            self._process_detection(msg)
        elif self.state == State.CENTERING:
            self._update_target(msg)
    
    def _process_detection(self, msg: Image):
        """Detection phase processing."""
        # Check timeout
        elapsed = self._get_state_elapsed()
        if elapsed > self.detection_timeout:
            self.get_logger().warn('Detection timeout - no disease found')
            self._publish_status(False)
            self._transition_to(State.IDLE)
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            bbox = self._detect_disease(cv_image)
            
            self.detection_attempts += 1
            
            if bbox is not None:
                self.consecutive_detections += 1
                self.current_bbox = bbox
                
                if self.consecutive_detections >= self.consec_required:
                    self.get_logger().info(
                        f'Disease detected after {self.detection_attempts} attempts'
                    )
                    self._publish_status(True)
                    self._transition_to(State.CENTERING)
            else:
                self.consecutive_detections = 0
            
            # Max attempts check
            if self.detection_attempts >= self.max_attempts and \
               self.consecutive_detections < self.consec_required:
                self.get_logger().warn('Detection failed after max attempts')
                self._publish_status(False)
                self._transition_to(State.IDLE)
                
        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')
    
    def _update_target(self, msg: Image):
        """Update target bbox during centering."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            bbox = self._detect_disease(cv_image)
            if bbox is not None:
                self.current_bbox = bbox
        except Exception as e:
            self.get_logger().error(f'Target update error: {e}')
    
    def control_loop(self):
        """Main control loop for centering."""
        if self.state != State.CENTERING:
            return
        
        # Check timeout
        elapsed = self._get_state_elapsed()
        if elapsed > self.centering_timeout:
            self.get_logger().warn('Centering timeout')
            self._stop_motion()
            self._publish_ready(False)
            self._transition_to(State.IDLE)
            return
        
        if self.current_bbox is None:
            return
        
        # Calculate bbox center
        x, y, w, h = self.current_bbox
        target_x = x + w / 2
        target_y = y + h / 2
        
        # Calculate error from image center
        error_x = target_x - self.center_x
        error_y = target_y - self.center_y
        
        # Check if centered
        if abs(error_x) < self.centered_thresh and abs(error_y) < self.centered_thresh:
            self.get_logger().info(
                f'Target centered (error: {error_x:.1f}px, {error_y:.1f}px)'
            )
            self._stop_motion()
            self._publish_ready(True)
            self._transition_to(State.COMPLETED)
            return
        
        # PID control
        self.pid.integral_x += error_x * self.dt
        self.pid.integral_y += error_y * self.dt
        
        deriv_x = (error_x - self.pid.prev_error_x) / self.dt
        deriv_y = (error_y - self.pid.prev_error_y) / self.dt
        
        # Calculate velocities (image X → body Y, image Y → body X)
        vel_y = -(self.kp * error_x + self.ki * self.pid.integral_x + self.kd * deriv_x)
        vel_x = -(self.kp * error_y + self.ki * self.pid.integral_y + self.kd * deriv_y)
        
        # Clamp velocities
        vel_x = max(-self.max_vel, min(self.max_vel, vel_x))
        vel_y = max(-self.max_vel, min(self.max_vel, vel_y))
        
        self.pid.prev_error_x = error_x
        self.pid.prev_error_y = error_y
        
        # Publish velocity command
        self._publish_velocity(vel_x, vel_y)
    
    def _detect_disease(self, image: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
        """Detect disease in image. Returns bbox (x, y, w, h) or None."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Look for brownish/yellowish disease spots
        lower = np.array([10, 50, 50])
        upper = np.array([30, 255, 200])
        mask = cv2.inRange(hsv, lower, upper)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Find largest contour
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        
        if area < self.min_area:
            return None
        
        x, y, w, h = cv2.boundingRect(largest)
        return (x, y, w, h)
    
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
    
    def _publish_status(self, detected: bool):
        """Publish detection status."""
        msg = Bool()
        msg.data = detected
        self.status_pub.publish(msg)
    
    def _transition_to(self, new_state: State):
        """Transition to new state."""
        old_state = self.state
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        
        # Reset state-specific variables
        if new_state == State.DETECTING:
            self.detection_attempts = 0
            self.consecutive_detections = 0
            self.current_bbox = None
        elif new_state == State.CENTERING:
            self.pid = PIDState()  # Reset PID
        elif new_state == State.IDLE:
            self._stop_motion()
            self.current_bbox = None
        
        self.get_logger().info(f'State: {old_state.name} → {new_state.name}')
    
    def _get_state_elapsed(self) -> float:
        """Get time elapsed in current state (seconds)."""
        if self.state_start_time is None:
            return 0.0
        return (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = DetectionCenteringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
