#!/usr/bin/env python3
# Copyright 2024 Shaan Shoukath
# SPDX-License-Identifier: Apache-2.0

"""
Image Capture Node

Camera abstraction layer for capturing and publishing raw images during
survey flight. Supports USB cameras via OpenCV.

Publishers:
    /camera/image_raw (sensor_msgs/Image): Raw camera images
    /camera/camera_info (sensor_msgs/CameraInfo): Camera calibration info

Subscribers (SITL only):
    /camera/inject_test_image (sensor_msgs/Image): Injected test images
"""

import cv2
import numpy as np
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class ImageCaptureNode(Node):
    """ROS2 node for camera image capture."""
    
    def __init__(self):
        super().__init__('image_capture_node')
        
        # ================================================================
        # PARAMETERS
        # ================================================================
        # Camera mode: use_sim selects between laptop webcam and Pi Camera
        self.declare_parameter('use_sim', True)  # True = laptop webcam, False = Pi Camera
        
        # Camera device settings
        self.declare_parameter('camera_device', '/dev/video0')  # USB camera path
        self.declare_parameter('webcam_index', 0)  # Laptop webcam index for SITL
        self.declare_parameter('pi_camera_device', '/dev/video0')  # Pi Camera via v4l2
        
        # Resolution and frame rate
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('frame_id', 'camera_optical_frame')
        
        # Camera matrix parameters
        self.declare_parameter('camera_matrix.fx', 1000.0)
        self.declare_parameter('camera_matrix.fy', 1000.0)
        self.declare_parameter('camera_matrix.cx', 960.0)
        self.declare_parameter('camera_matrix.cy', 540.0)
        
        # Distortion parameters
        self.declare_parameter('distortion_coefficients.k1', 0.0)
        self.declare_parameter('distortion_coefficients.k2', 0.0)
        self.declare_parameter('distortion_coefficients.p1', 0.0)
        self.declare_parameter('distortion_coefficients.p2', 0.0)
        self.declare_parameter('distortion_coefficients.k3', 0.0)
        
        # Get parameters
        self.use_sim = self.get_parameter('use_sim').value
        self.camera_device = self.get_parameter('camera_device').value
        self.webcam_index = self.get_parameter('webcam_index').value
        self.pi_camera_device = self.get_parameter('pi_camera_device').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Camera intrinsics
        self.fx = self.get_parameter('camera_matrix.fx').value
        self.fy = self.get_parameter('camera_matrix.fy').value
        self.cx = self.get_parameter('camera_matrix.cx').value
        self.cy = self.get_parameter('camera_matrix.cy').value
        
        self.k1 = self.get_parameter('distortion_coefficients.k1').value
        self.k2 = self.get_parameter('distortion_coefficients.k2').value
        self.p1 = self.get_parameter('distortion_coefficients.p1').value
        self.p2 = self.get_parameter('distortion_coefficients.p2').value
        self.k3 = self.get_parameter('distortion_coefficients.k3').value
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # Test image injection (SITL mode only)
        self.injected_frame: Optional[np.ndarray] = None
        
        # Camera capture
        self.cap: Optional[cv2.VideoCapture] = None
        self._init_camera()
        
        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.image_pub = self.create_publisher(
            Image,
            '/camera/image_raw',
            sensor_qos
        )
        
        self.info_pub = self.create_publisher(
            CameraInfo,
            '/camera/camera_info',
            sensor_qos
        )
        
        # Create camera info message (static)
        self.camera_info_msg = self._create_camera_info()
        
        # Test image injection subscriber (SITL mode only)
        if self.use_sim:
            self.inject_sub = self.create_subscription(
                Image,
                '/camera/inject_test_image',
                self.inject_callback,
                10
            )
            self.get_logger().info('  Test injection: ENABLED (/camera/inject_test_image)')
        
        # Capture timer
        self.capture_timer = self.create_timer(
            1.0 / self.fps,
            self.capture_callback
        )
        
        self.get_logger().info('Image Capture Node initialized')
        self.get_logger().info(f'  Mode: {"SITL (laptop webcam)" if self.use_sim else "Pi Camera"}')
        self.get_logger().info(f'  Resolution: {self.image_width}x{self.image_height}')
        self.get_logger().info(f'  FPS: {self.fps}')
    
    def _init_camera(self):
        """
        Initialize camera capture based on use_sim mode.
        
        Modes:
            use_sim=True  → Laptop webcam via index (SITL testing)
            use_sim=False → Pi Camera 3 via libcamera/v4l2 (real hardware)
        """
        try:
            if self.use_sim:
                # SITL mode: Use laptop webcam by index
                self.get_logger().info(f'Opening laptop webcam (index {self.webcam_index})...')
                self.cap = cv2.VideoCapture(self.webcam_index)
                source_name = f'webcam:{self.webcam_index}'
            else:
                # Real hardware: Pi Camera 3 Wide via libcamera
                # Pi Camera 3 on Pi 5 is accessible via libcamera-vid or v4l2
                # Use GStreamer pipeline for libcamera on Pi 5
                pi_cam_pipeline = (
                    f'libcamerasrc ! '
                    f'video/x-raw,width={self.image_width},height={self.image_height},framerate={int(self.fps)}/1 ! '
                    f'videoconvert ! appsink'
                )
                
                # Try libcamera first (Pi 5 + Pi Camera 3)
                self.get_logger().info('Trying libcamera GStreamer pipeline...')
                self.cap = cv2.VideoCapture(pi_cam_pipeline, cv2.CAP_GSTREAMER)
                
                if not self.cap.isOpened():
                    # Fallback to v4l2 device
                    self.get_logger().warn('libcamera failed, trying v4l2 device...')
                    self.cap = cv2.VideoCapture(self.pi_camera_device)
                
                source_name = 'Pi Camera (libcamera/v4l2)'
            
            if not self.cap.isOpened():
                self.get_logger().error(f'Failed to open camera')
                self.get_logger().info('Falling back to simulated test images')
                self.cap = None
                return
            
            # Set resolution (may not apply to all cameras)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            # Verify actual settings
            actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            
            self.get_logger().info(
                f'Camera opened: {source_name} ({int(actual_width)}x{int(actual_height)})'
            )
            
        except Exception as e:
            self.get_logger().error(f'Camera initialization error: {e}')
            self.get_logger().info('Falling back to simulated test images')
            self.cap = None
    
    def _create_camera_info(self) -> CameraInfo:
        """Create CameraInfo message."""
        msg = CameraInfo()
        msg.header.frame_id = self.frame_id
        msg.width = self.image_width
        msg.height = self.image_height
        
        # Camera matrix (3x3)
        msg.k = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0
        ]
        
        # Distortion coefficients
        msg.d = [self.k1, self.k2, self.p1, self.p2, self.k3]
        msg.distortion_model = 'plumb_bob'
        
        # Rectification matrix (identity for monocular)
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # Projection matrix (3x4)
        msg.p = [
            self.fx, 0.0, self.cx, 0.0,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        return msg
    
    def capture_callback(self):
        """Capture and publish camera frame."""
        timestamp = self.get_clock().now().to_msg()
        
        # Check for injected test image first (SITL mode)
        if self.injected_frame is not None:
            frame = self.injected_frame
            self.injected_frame = None  # Clear after use (one-shot)
            self.get_logger().info('📸 Publishing injected test image')
        elif self.cap is not None and self.cap.isOpened():
            ret, frame = self.cap.read()
            
            if not ret:
                self.get_logger().warn('Failed to capture frame')
                return
        else:
            # Generate simulated image for testing
            frame = self._generate_test_image()
        
        # Convert to ROS message
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image_msg.header = Header()
            image_msg.header.stamp = timestamp
            image_msg.header.frame_id = self.frame_id
            
            # Update camera info timestamp
            self.camera_info_msg.header.stamp = timestamp
            
            # Publish
            self.image_pub.publish(image_msg)
            self.info_pub.publish(self.camera_info_msg)
            
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
    
    def inject_callback(self, msg: Image):
        """
        Receive injected test image for SITL testing.
        
        When an image is received on /camera/inject_test_image, it will be
        published as the next frame on /camera/image_raw instead of camera feed.
        """
        try:
            self.injected_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.get_logger().info(
                f'🎯 Test image received ({self.injected_frame.shape[1]}x{self.injected_frame.shape[0]})'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to convert injected image: {e}')
    
    def _generate_test_image(self) -> np.ndarray:
        """Generate test image for simulation."""
        # Create gradient test image
        img = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        
        # Add gradient background
        for y in range(self.image_height):
            img[y, :, 1] = int(100 * (y / self.image_height))  # Green gradient
        
        # Add some "crop" patterns
        for i in range(10):
            x = int((i + 0.5) * self.image_width / 10)
            cv2.line(img, (x, 0), (x, self.image_height), (0, 150, 0), 2)
        
        # Add timestamp
        timestamp_str = f"SIM: {self.get_clock().now().nanoseconds}"
        cv2.putText(img, timestamp_str, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        return img
    
    def destroy_node(self):
        """Clean up resources."""
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = ImageCaptureNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
