#!/usr/bin/env python3
# Copyright 2024 Maintainer
# SPDX-License-Identifier: Apache-2.0

"""
Local Detection Node

Confirms disease presence locally upon arrival at target.
Provides bounding box for centering controller.

Subscribers:
    /drone2/arrival_status (std_msgs/Bool): Trigger to start detection
    /camera/image_raw (sensor_msgs/Image): Camera feed
    /mavros/local_position/pose (geometry_msgs/PoseStamped): Current pose

Publishers:
    /drone2/local_bbox (vision_msgs/BoundingBox2D): Detection bounding box
    /drone2/local_detection_status (std_msgs/Bool): Detection confirmed
"""

import cv2
import numpy as np
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import BoundingBox2D
from cv_bridge import CvBridge


class LocalDetectionNode(Node):
    """ROS2 node for local disease confirmation."""
    
    def __init__(self):
        super().__init__('local_detection_node')
        
        # Declare parameters
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('detection_timeout_sec', 30.0)
        self.declare_parameter('max_detection_attempts', 5)
        self.declare_parameter('require_consecutive_detections', 2)
        
        # Get parameters
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.timeout = self.get_parameter('detection_timeout_sec').value
        self.max_attempts = self.get_parameter('max_detection_attempts').value
        self.consec_required = self.get_parameter('require_consecutive_detections').value
        
        # State
        self.detecting = False
        self.detection_start_time = None
        self.detection_attempts = 0
        self.consecutive_detections = 0
        self.last_bbox: Optional[Tuple[int, int, int, int]] = None
        
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
        self.bbox_pub = self.create_publisher(
            BoundingBox2D, '/drone2/local_bbox', 10
        )
        
        self.status_pub = self.create_publisher(
            Bool, '/drone2/local_detection_status', reliable_qos
        )
        
        self.get_logger().info('Local Detection Node initialized')
    
    def arrival_callback(self, msg: Bool):
        """Start detection when arrived at target."""
        if msg.data and not self.detecting:
            self.detecting = True
            self.detection_start_time = self.get_clock().now()
            self.detection_attempts = 0
            self.consecutive_detections = 0
            self.get_logger().info('Starting local detection')
    
    def pose_callback(self, msg: PoseStamped):
        """Update current pose (for future enhancements)."""
        pass
    
    def image_callback(self, msg: Image):
        """Process image for local detection."""
        if not self.detecting:
            return
        
        # Check timeout
        elapsed = (self.get_clock().now() - self.detection_start_time).nanoseconds / 1e9
        if elapsed > self.timeout:
            self.get_logger().warn('Detection timeout')
            self._publish_status(False)
            self.detecting = False
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            bbox = self._detect_disease(cv_image)
            
            self.detection_attempts += 1
            
            if bbox is not None:
                self.consecutive_detections += 1
                self.last_bbox = bbox
                self._publish_bbox(bbox, msg.header)
                
                if self.consecutive_detections >= self.consec_required:
                    self.get_logger().info('Disease confirmed locally')
                    self._publish_status(True)
                    self.detecting = False
            else:
                self.consecutive_detections = 0
            
            if self.detection_attempts >= self.max_attempts and \
               self.consecutive_detections < self.consec_required:
                self.get_logger().warn('Detection failed after max attempts')
                self._publish_status(False)
                self.detecting = False
                
        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')
    
    def _detect_disease(self, image: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
        """Detect disease in image. Returns bbox (x, y, w, h) or None."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Look for brownish/yellowish spots
        lower = np.array([10, 50, 50])
        upper = np.array([30, 255, 200])
        mask = cv2.inRange(hsv, lower, upper)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Find largest contour
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        
        if area < 500:  # Minimum area
            return None
        
        x, y, w, h = cv2.boundingRect(largest)
        return (x, y, w, h)
    
    def _publish_bbox(self, bbox: Tuple[int, int, int, int], header):
        """Publish bounding box."""
        x, y, w, h = bbox
        msg = BoundingBox2D()
        msg.center.position.x = float(x + w / 2)
        msg.center.position.y = float(y + h / 2)
        msg.size_x = float(w)
        msg.size_y = float(h)
        self.bbox_pub.publish(msg)
    
    def _publish_status(self, detected: bool):
        """Publish detection status."""
        msg = Bool()
        msg.data = detected
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
