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
from mavros_msgs.msg import StatusText
from cv_bridge import CvBridge


class State(Enum):
    """Detection and centering state machine."""
    IDLE = auto()         # Waiting for arrival trigger
    DETECTING = auto()    # Looking for disease
    CENTERING = auto()    # Visual servoing to center
    DESCENDING = auto()   # Descending to spray altitude
    SPRAYING = auto()     # Waiting for spray to complete
    ASCENDING = auto()    # Ascending back to navigation altitude
    COMPLETED = auto()    # Cycle complete, ready for next target


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
        
        # Simulation mode - skip detection, go straight to descent
        self.declare_parameter('use_sim', True)
        
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
        
        # Camera offset from drone center (cm)
        # Positive X = camera is RIGHT of center
        # Positive Y = camera is FORWARD of center
        self.declare_parameter('camera_offset_x_cm', 0.0)
        self.declare_parameter('camera_offset_y_cm', 0.0)
        
        # Camera FOV and altitude for automatic calibration
        self.declare_parameter('camera_hfov_deg', 62.0)  # Pi Camera horizontal FOV
        self.declare_parameter('camera_vfov_deg', 48.0)  # Pi Camera vertical FOV
        self.declare_parameter('centering_altitude_m', 6.7)  # 22 feet navigation altitude
        
        # Altitude parameters (EASILY CHANGEABLE - 22 feet = 6.7m)
        self.declare_parameter('navigation_altitude_m', 6.7)  # Height for navigation (22 feet)
        self.declare_parameter('spray_altitude_m', 2.0)       # Target altitude for spraying
        self.declare_parameter('descent_velocity_mps', 0.3)   # Descent speed
        self.declare_parameter('ascend_velocity_mps', 0.5)    # Ascend speed (faster than descent)
        self.declare_parameter('altitude_tolerance_m', 0.3)   # Tolerance for altitude checks
        self.declare_parameter('spray_duration_sec', 3.0)     # How long to spray
        
        # Simulation mode (skip detection/centering)
        self.use_sim = self.get_parameter('use_sim').value
        
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
        
        # Spray altitude params (easily accessible)
        self.navigation_altitude = self.get_parameter('navigation_altitude_m').value
        self.spray_altitude = self.get_parameter('spray_altitude_m').value
        self.descent_velocity = self.get_parameter('descent_velocity_mps').value
        self.ascend_velocity = self.get_parameter('ascend_velocity_mps').value
        self.altitude_tolerance = self.get_parameter('altitude_tolerance_m').value
        self.spray_duration = self.get_parameter('spray_duration_sec').value
        
        # Camera offset and calibration
        cam_offset_x = self.get_parameter('camera_offset_x_cm').value
        cam_offset_y = self.get_parameter('camera_offset_y_cm').value
        hfov_deg = self.get_parameter('camera_hfov_deg').value
        vfov_deg = self.get_parameter('camera_vfov_deg').value
        altitude_m = self.get_parameter('centering_altitude_m').value
        
        # Calculate ground coverage at centering altitude
        # ground_width = 2 * altitude * tan(hfov/2)
        import math
        hfov_rad = math.radians(hfov_deg)
        vfov_rad = math.radians(vfov_deg)
        ground_width_m = 2 * altitude_m * math.tan(hfov_rad / 2)
        ground_height_m = 2 * altitude_m * math.tan(vfov_rad / 2)
        
        # pixels per cm = image_dimension / ground_dimension_cm
        pixels_per_cm_x = self.image_width / (ground_width_m * 100)
        pixels_per_cm_y = self.image_height / (ground_height_m * 100)
        pixels_per_cm = (pixels_per_cm_x + pixels_per_cm_y) / 2  # Average
        
        # Calculate adjusted center (shift by camera offset)
        # If camera is 5cm RIGHT of center, we need target at image center to end up 5cm LEFT in real world
        # So we SHIFT the target point in the opposite direction
        offset_x_pixels = cam_offset_x * pixels_per_cm_x
        offset_y_pixels = cam_offset_y * pixels_per_cm_y
        
        self.center_x = (self.image_width / 2) - offset_x_pixels
        self.center_y = (self.image_height / 2) - offset_y_pixels
        
        self.get_logger().info(f'Camera: FOV={hfov_deg}x{vfov_deg}deg, altitude={altitude_m}m')
        self.get_logger().info(f'Ground coverage: {ground_width_m:.2f}x{ground_height_m:.2f}m ({pixels_per_cm:.1f} px/cm)')
        self.get_logger().info(f'Camera offset: X={cam_offset_x}cm, Y={cam_offset_y}cm')
        self.get_logger().info(f'Adjusted center: ({self.center_x:.1f}, {self.center_y:.1f})')
        self.get_logger().info(f'Spray altitude: {self.spray_altitude}m (descent speed: {self.descent_velocity}m/s)')
        
        # State machine
        self.state = State.IDLE
        self.state_start_time = None
        
        # Detection state
        self.detection_attempts = 0
        self.consecutive_detections = 0
        self.current_bbox: Optional[Tuple[int, int, int, int]] = None
        
        # Altitude tracking
        self.current_altitude: float = 0.0
        
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
        # Use sensor_qos for arrival_status for compatibility with ros2 topic pub
        self.arrival_sub = self.create_subscription(
            Bool, '/drone2/arrival_status',
            self.arrival_callback, sensor_qos
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
        
        # MAVLink status text (visible in Mission Planner)
        self.statustext_pub = self.create_publisher(
            StatusText, '/mavros/statustext/send', 10
        )
        
        # Control timer
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info('Detection & Centering Node initialized (MERGED)')
        self.get_logger().info(f'  Detection timeout: {self.detection_timeout}s')
        self.get_logger().info(f'  Centering threshold: {self.centered_thresh}px')
        self.get_logger().info(f'  Max velocity: {self.max_vel} m/s')
    
    def arrival_callback(self, msg: Bool):
        """Start detection/spray cycle when arrived at target."""
        if msg.data and self.state == State.IDLE:
            if self.use_sim:
                # SITL mode: skip detection/centering, go straight to descent
                self.get_logger().info('Arrived (SIM MODE) - skipping detection, descending to spray')
                self._transition_to(State.DESCENDING)
            else:
                # Real mode: run full detection cycle
                self._transition_to(State.DETECTING)
                self.get_logger().info('Arrived - starting detection')
    
    def pose_callback(self, msg: PoseStamped):
        """Update current altitude from local position."""
        self.current_altitude = msg.pose.position.z
    
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
        """Main control loop for centering, descent, spray, and ascent."""
        if self.state == State.CENTERING:
            self._handle_centering()
        elif self.state == State.DESCENDING:
            self._handle_descent()
        elif self.state == State.SPRAYING:
            self._handle_spraying()
        elif self.state == State.ASCENDING:
            self._handle_ascent()
    
    def _handle_centering(self):
        """Handle centering phase - visual servoing to align with target."""
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
        
        # Check if centered - transition to DESCENDING
        if abs(error_x) < self.centered_thresh and abs(error_y) < self.centered_thresh:
            self.get_logger().info(
                f'Target centered (error: {error_x:.1f}px, {error_y:.1f}px) - descending to spray altitude'
            )
            self._stop_motion()
            self._transition_to(State.DESCENDING)
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
    
    def _handle_descent(self):
        """Handle descent phase - descend to spray altitude."""
        # Log progress every second
        elapsed = self._get_state_elapsed()
        if int(elapsed) != getattr(self, '_last_descent_log', -1):
            self._last_descent_log = int(elapsed)
            self.get_logger().info(
                f'[DESCENDING] Current: {self.current_altitude:.1f}m → Target: {self.spray_altitude:.1f}m'
            )
        
        # Check if at spray altitude
        if self.current_altitude <= self.spray_altitude + self.altitude_tolerance:
            self.get_logger().info(
                f'Reached spray altitude: {self.current_altitude:.1f}m - starting spray'
            )
            self._stop_motion()
            self._publish_ready(True)  # Signal sprayer to activate
            self._transition_to(State.SPRAYING)
            return
        
        # Descent timeout (30 seconds max)
        if elapsed > 30.0:
            self.get_logger().warn('Descent timeout')
            self._stop_motion()
            self._publish_ready(False)
            self._transition_to(State.IDLE)
            return
        
        # Command descent velocity
        self._publish_velocity(0.0, 0.0, -self.descent_velocity)
    
    def _handle_spraying(self):
        """Handle spray phase - wait for spray duration then ascend."""
        elapsed = self._get_state_elapsed()
        
        # Log every second
        if int(elapsed) != getattr(self, '_last_spray_log', -1):
            self._last_spray_log = int(elapsed)
            remaining = max(0, self.spray_duration - elapsed)
            self.get_logger().info(f'[SPRAYING] {remaining:.0f}s remaining...')
        
        # Spray complete - start ascending
        if elapsed >= self.spray_duration:
            self.get_logger().info('Spray complete - ascending to navigation altitude')
            self._transition_to(State.ASCENDING)
    
    def _handle_ascent(self):
        """Handle ascent phase - climb back to navigation altitude."""
        elapsed = self._get_state_elapsed()
        
        # Log progress every second
        if int(elapsed) != getattr(self, '_last_ascent_log', -1):
            self._last_ascent_log = int(elapsed)
            self.get_logger().info(
                f'[ASCENDING] Current: {self.current_altitude:.1f}m → Target: {self.navigation_altitude:.1f}m'
            )
        
        # Check if at navigation altitude
        if self.current_altitude >= self.navigation_altitude - self.altitude_tolerance:
            self.get_logger().info(
                f'Reached navigation altitude: {self.current_altitude:.1f}m - ready for next target'
            )
            self._stop_motion()
            self._transition_to(State.COMPLETED)
            return
        
        # Ascent timeout (30 seconds max)
        if elapsed > 30.0:
            self.get_logger().warn('Ascent timeout')
            self._stop_motion()
            self._transition_to(State.COMPLETED)  # Still continue to next target
            return
        
        # Command ascent velocity
        self._publish_velocity(0.0, 0.0, self.ascend_velocity)
    
    def _detect_disease(self, image: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
        """Detect disease in image. Returns bbox (x, y, w, h) or None."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Look for yellow disease (synced with Drone-1 range)
        lower = np.array([20, 50, 50])   # H:20-35, S:50-255, V:50-255
        upper = np.array([35, 255, 255])
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
    
    def _publish_velocity(self, vx: float, vy: float, vz: float = 0.0):
        """Publish velocity command (body frame)."""
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        
        self.vel_pub.publish(msg)
    
    def _stop_motion(self):
        """Stop all motion."""
        self._publish_velocity(0.0, 0.0, 0.0)
    
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
    
    def _send_statustext(self, text: str, severity: int = 6):
        """
        Send status text to Mission Planner via MAVLink.
        
        Severity levels:
            0 = EMERGENCY, 1 = ALERT, 2 = CRITICAL, 3 = ERROR
            4 = WARNING, 5 = NOTICE, 6 = INFO, 7 = DEBUG
        """
        msg = StatusText()
        msg.severity = severity
        msg.text = text[:50]  # MAVLink limit is 50 chars
        self.statustext_pub.publish(msg)
    
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
        
        # Log state change
        self.get_logger().info(f'State: {old_state.name} → {new_state.name}')
        
        # Send to Mission Planner
        status_messages = {
            State.DETECTING: "D2: Detecting disease...",
            State.CENTERING: "D2: Centering on target",
            State.DESCENDING: "D2: Descending to spray",
            State.SPRAYING: "D2: SPRAYING",
            State.ASCENDING: "D2: Ascending to nav alt",
            State.COMPLETED: "D2: Spray complete!",
            State.IDLE: "D2: Ready for target",
        }
        if new_state in status_messages:
            self._send_statustext(status_messages[new_state])
    
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
