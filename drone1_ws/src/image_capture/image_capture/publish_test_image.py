#!/usr/bin/env python3
# Copyright 2024 Shaan Shoukath
# SPDX-License-Identifier: Apache-2.0

"""
Publish Test Image

CLI tool to publish a test image with yellow spots to trigger
disease detection during SITL simulation.

Usage:
    # Generate and publish a synthetic yellow test image
    ros2 run image_capture publish_test_image
    
    # Publish a custom image file
    ros2 run image_capture publish_test_image --image /path/to/image.png
"""

import argparse
import sys

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class TestImagePublisher(Node):
    """One-shot publisher for test images."""
    
    def __init__(self, image_path: str = None):
        super().__init__('test_image_publisher')
        
        self.bridge = CvBridge()
        
        # Publisher
        self.pub = self.create_publisher(
            Image,
            '/camera/inject_test_image',
            10
        )
        
        # Generate or load image
        if image_path:
            self.get_logger().info(f'Loading image from: {image_path}')
            self.frame = cv2.imread(image_path)
            if self.frame is None:
                self.get_logger().error(f'Failed to load image: {image_path}')
                sys.exit(1)
        else:
            self.get_logger().info('Generating synthetic yellow test image...')
            self.frame = self._generate_yellow_test_image()
        
        # Publish after short delay to ensure connection
        self.timer = self.create_timer(0.5, self._publish_and_exit)
    
    def _generate_yellow_test_image(self) -> np.ndarray:
        """
        Generate a test image with green background and yellow spots.
        
        Creates an image that will trigger disease detection:
        - Green vegetation background (to pass the 'near green' check)
        - Yellow spots (to trigger yellow detection)
        """
        width, height = 1280, 720
        
        # Create green gradient background (simulates crop field)
        img = np.zeros((height, width, 3), dtype=np.uint8)
        for y in range(height):
            green_val = 80 + int(100 * (y / height))
            img[y, :] = [30, green_val, 20]  # BGR: dark to bright green
        
        # Add some "crop row" lines
        for i in range(10):
            x = int((i + 0.5) * width / 10)
            cv2.line(img, (x, 0), (x, height), (20, 100, 15), 3)
        
        # Add yellow disease spots (bright yellow - HSV: H=25-35, S=200+, V=200+)
        yellow_spots = [
            (320, 180, 50),   # (x, y, radius)
            (640, 360, 70),   # Larger spot in center
            (960, 540, 40),
            (400, 500, 35),
            (850, 200, 45),
        ]
        
        for x, y, r in yellow_spots:
            # Bright yellow: BGR = (0, 255, 255)
            cv2.circle(img, (x, y), r, (0, 255, 255), -1)
            # Add slight color variation for realism
            cv2.circle(img, (x+5, y-5), r//2, (20, 230, 255), -1)
        
        # Add text label
        cv2.putText(img, 'TEST IMAGE - Yellow Disease Spots', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(img, 'Green background + Yellow spots = Detection trigger', (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        
        return img
    
    def _publish_and_exit(self):
        """Publish image and shutdown."""
        try:
            msg = self.bridge.cv2_to_imgmsg(self.frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_optical_frame'
            
            self.pub.publish(msg)
            
            h, w = self.frame.shape[:2]
            self.get_logger().info(f'✅ Published test image ({w}x{h}) to /camera/inject_test_image')
            self.get_logger().info('🎯 Detection node should now process this image')
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish: {e}')
        
        # Shutdown after publishing
        self.timer.cancel()
        raise SystemExit(0)


def main(args=None):
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Publish test image to trigger disease detection'
    )
    parser.add_argument(
        '--image', '-i',
        type=str,
        default=None,
        help='Path to custom image file (default: generate synthetic image)'
    )
    
    # Parse known args to allow ROS2 args
    parsed_args, remaining = parser.parse_known_args()
    
    rclpy.init(args=remaining)
    
    node = TestImagePublisher(image_path=parsed_args.image)
    
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
