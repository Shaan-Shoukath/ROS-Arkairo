#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════
PLANT DISEASE DETECTION AND GEOTAGGING NODE FOR ROS2
═══════════════════════════════════════════════════════════════════════════════

Detects yellow disease in plants using computer vision and computes GPS geotags
using ray-casting from drone altitude and IMU orientation.

FEATURES:
  ✓ Yellow disease detection with optimized HSV color ranges
  ✓ Green vegetation context validation
  ✓ Sand/soil exclusion filtering
  ✓ Shape-based validation (aspect ratio, compactness)
  ✓ GPS geotagging with deduplication
  ✓ Severity classification (MILD/MODERATE/SEVERE)
  ✓ Ray-casting GPS projection using IMU and camera intrinsics

Subscribers:
    /camera/image_raw (sensor_msgs/Image): Camera frames
    /mavros/global_position/global (sensor_msgs/NavSatFix): GPS position
    /mavros/imu/data (sensor_msgs/Imu): IMU orientation
    /mavros/local_position/pose (geometry_msgs/PoseStamped): Altitude

Publishers:
    /drone1/disease_geotag (geographic_msgs/GeoPointStamped): Detection GPS
    /drone1/detection_debug (sensor_msgs/Image): Debug visualization
═══════════════════════════════════════════════════════════════════════════════
"""

import math
import csv
import os
from datetime import datetime
from typing import List, Tuple, Optional
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Header, Bool
from sensor_msgs.msg import Image, Imu, NavSatFix
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPointStamped, GeoPoint

from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    from scipy.spatial.transform import Rotation as R
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


@dataclass
class Detection:
    """Detection result with GPS and metadata."""
    latitude: float
    longitude: float
    pixel_x: int
    pixel_y: int
    area: int
    severity: str
    severity_score: int
    confidence: float


class DetectionAndGeotagNode(Node):
    """ROS2 node for disease detection and GPS geotagging."""
    
    def __init__(self):
        super().__init__('detection_and_geotag_node')
        
        # ═══════════════════════════════════════════════════════════════════
        # PARAMETERS - Camera Intrinsics
        # ═══════════════════════════════════════════════════════════════════
        self.declare_parameter('camera_fx', 430.0)
        self.declare_parameter('camera_fy', 430.0)
        self.declare_parameter('camera_cx', 640.0)
        self.declare_parameter('camera_cy', 360.0)
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        
        # Camera orientation (pointing down)
        self.declare_parameter('camera_roll', 0.0)
        self.declare_parameter('camera_pitch', 1.5708)  # 90 degrees
        self.declare_parameter('camera_yaw', 0.0)
        
        # Camera FOV for simple projection fallback
        self.declare_parameter('hfov_deg', 60.0)
        self.declare_parameter('vfov_deg', 45.0)
        
        # ═══════════════════════════════════════════════════════════════════
        # PARAMETERS - Yellow Disease Detection HSV
        # ═══════════════════════════════════════════════════════════════════
        self.declare_parameter('yellow_h_min', 15)
        self.declare_parameter('yellow_h_max', 40)
        self.declare_parameter('yellow_s_min', 80)
        self.declare_parameter('yellow_s_max', 255)
        self.declare_parameter('yellow_v_min', 80)
        self.declare_parameter('yellow_v_max', 255)
        
        # Green vegetation detection (plant context)
        self.declare_parameter('green_h_min', 35)
        self.declare_parameter('green_h_max', 85)
        self.declare_parameter('green_s_min', 30)
        self.declare_parameter('green_s_max', 255)
        self.declare_parameter('green_v_min', 30)
        self.declare_parameter('green_v_max', 255)
        
        # Brown/sand exclusion
        self.declare_parameter('brown_h_min', 5)
        self.declare_parameter('brown_h_max', 18)
        self.declare_parameter('brown_s_min', 10)
        self.declare_parameter('brown_s_max', 100)
        self.declare_parameter('brown_v_min', 40)
        self.declare_parameter('brown_v_max', 180)
        
        # ═══════════════════════════════════════════════════════════════════
        # PARAMETERS - Detection Thresholds
        # ═══════════════════════════════════════════════════════════════════
        self.declare_parameter('min_detection_area', 250)
        self.declare_parameter('min_green_nearby', 150)
        self.declare_parameter('gps_dedup_distance_m', 3.0)
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('log_to_csv', True)
        self.declare_parameter('csv_log_path', '/tmp/disease_detections.csv')
        
        # ═══════════════════════════════════════════════════════════════════
        # GET PARAMETERS
        # ═══════════════════════════════════════════════════════════════════
        self.fx = self.get_parameter('camera_fx').value
        self.fy = self.get_parameter('camera_fy').value
        self.cx = self.get_parameter('camera_cx').value
        self.cy = self.get_parameter('camera_cy').value
        self.img_width = self.get_parameter('image_width').value
        self.img_height = self.get_parameter('image_height').value
        
        self.cam_roll = self.get_parameter('camera_roll').value
        self.cam_pitch = self.get_parameter('camera_pitch').value
        self.cam_yaw = self.get_parameter('camera_yaw').value
        
        self.hfov = math.radians(self.get_parameter('hfov_deg').value)
        self.vfov = math.radians(self.get_parameter('vfov_deg').value)
        
        # Yellow HSV
        self.yellow_min = np.array([
            self.get_parameter('yellow_h_min').value,
            self.get_parameter('yellow_s_min').value,
            self.get_parameter('yellow_v_min').value
        ])
        self.yellow_max = np.array([
            self.get_parameter('yellow_h_max').value,
            self.get_parameter('yellow_s_max').value,
            self.get_parameter('yellow_v_max').value
        ])
        
        # Green HSV
        self.green_min = np.array([
            self.get_parameter('green_h_min').value,
            self.get_parameter('green_s_min').value,
            self.get_parameter('green_v_min').value
        ])
        self.green_max = np.array([
            self.get_parameter('green_h_max').value,
            self.get_parameter('green_s_max').value,
            self.get_parameter('green_v_max').value
        ])
        
        # Brown HSV
        self.brown_min = np.array([
            self.get_parameter('brown_h_min').value,
            self.get_parameter('brown_s_min').value,
            self.get_parameter('brown_v_min').value
        ])
        self.brown_max = np.array([
            self.get_parameter('brown_h_max').value,
            self.get_parameter('brown_s_max').value,
            self.get_parameter('brown_v_max').value
        ])
        
        self.min_area = self.get_parameter('min_detection_area').value
        self.min_green = self.get_parameter('min_green_nearby').value
        self.dedup_distance = self.get_parameter('gps_dedup_distance_m').value
        self.publish_debug = self.get_parameter('publish_debug').value
        self.log_csv = self.get_parameter('log_to_csv').value
        self.csv_path = self.get_parameter('csv_log_path').value
        
        # ═══════════════════════════════════════════════════════════════════
        # STATE VARIABLES
        # ═══════════════════════════════════════════════════════════════════
        self.current_gps: Optional[NavSatFix] = None
        self.current_imu: Optional[Imu] = None
        self.current_altitude: float = 10.0
        self.logged_locations: List[Tuple[float, float]] = []
        self.detection_counter = 0
        self.detection_enabled = True  # Start enabled, nav node controls this
        
        # Compute camera-to-body rotation
        if HAS_SCIPY:
            rot = R.from_euler('xyz', [self.cam_roll, self.cam_pitch, self.cam_yaw])
            self.R_body_cam = rot.as_matrix()
        else:
            self.R_body_cam = None
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Morphological kernels
        self.kernel_small = np.ones((3, 3), np.uint8)
        self.kernel_large = np.ones((5, 5), np.uint8)
        
        # ═══════════════════════════════════════════════════════════════════
        # QOS PROFILES
        # ═══════════════════════════════════════════════════════════════════
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
        
        # ═══════════════════════════════════════════════════════════════════
        # SUBSCRIBERS
        # ═══════════════════════════════════════════════════════════════════
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw',
            self.image_callback, sensor_qos
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global',
            self.gps_callback, sensor_qos
        )
        
        self.imu_sub = self.create_subscription(
            Imu, '/mavros/imu/data',
            self.imu_callback, sensor_qos
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.pose_callback, sensor_qos
        )
        
        # Detection enable control from navigation
        self.detection_enable_sub = self.create_subscription(
            Bool, '/drone1/detection_enable',
            self.detection_enable_callback, 10
        )
        
        # ═══════════════════════════════════════════════════════════════════
        # PUBLISHERS
        # ═══════════════════════════════════════════════════════════════════
        self.geotag_pub = self.create_publisher(
            GeoPointStamped, '/drone1/disease_geotag', reliable_qos
        )
        
        self.debug_pub = self.create_publisher(
            Image, '/drone1/detection_debug', sensor_qos
        )
        
        # Initialize CSV
        if self.log_csv:
            self._init_csv()
        
        self.get_logger().info('Detection and Geotag Node initialized')
        self.get_logger().info(f'  Yellow HSV: {self.yellow_min} - {self.yellow_max}')
        self.get_logger().info(f'  Min area: {self.min_area}px')
        self.get_logger().info(f'  Dedup distance: {self.dedup_distance}m')
    
    def _init_csv(self):
        """Initialize CSV log file with headers."""
        if not os.path.isfile(self.csv_path):
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'Timestamp', 'Detection_ID', 'Latitude', 'Longitude',
                    'Altitude_m', 'Severity', 'Severity_Score', 'Area_px',
                    'Pixel_X', 'Pixel_Y', 'Status'
                ])
    
    def gps_callback(self, msg: NavSatFix):
        """Update current GPS position."""
        self.current_gps = msg
    
    def imu_callback(self, msg: Imu):
        """Update current IMU orientation."""
        self.current_imu = msg
    
    def pose_callback(self, msg: PoseStamped):
        """Update current altitude."""
        self.current_altitude = msg.pose.position.z
    
    def detection_enable_callback(self, msg: Bool):
        """Control detection from navigation node."""
        if msg.data != self.detection_enabled:
            self.detection_enabled = msg.data
            status = 'ENABLED' if msg.data else 'DISABLED'
            self.get_logger().info(f'Detection {status} by navigation')
    
    def image_callback(self, msg: Image):
        """Process camera frame for disease detection."""
        if self.current_gps is None:
            return
        
        # Only detect when enabled by navigation node
        if not self.detection_enabled:
            return
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w = frame.shape[:2]
            
            # Detect diseases
            detections = self._detect_yellow_disease(frame)
            
            # Process and publish each detection
            for det in detections:
                self._publish_geotag(det)
                
                if self.log_csv:
                    self._log_to_csv(det)
            
            # Publish debug image
            if self.publish_debug:
                debug_frame = self._draw_detections(frame, detections)
                self._publish_debug_image(debug_frame)
                
        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')
    
    def _detect_yellow_disease(self, frame: np.ndarray) -> List[Detection]:
        """
        Detect yellow disease using HSV color thresholding with validation.
        
        Returns list of validated Detection objects.
        """
        h, w = frame.shape[:2]
        detections = []
        
        # ═══════════════════════════════════════════════════════════════════
        # STEP 1: Convert to HSV
        # ═══════════════════════════════════════════════════════════════════
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # ═══════════════════════════════════════════════════════════════════
        # STEP 2: Create color masks
        # ═══════════════════════════════════════════════════════════════════
        yellow_mask = cv2.inRange(hsv, self.yellow_min, self.yellow_max)
        green_mask = cv2.inRange(hsv, self.green_min, self.green_max)
        brown_mask = cv2.inRange(hsv, self.brown_min, self.brown_max)
        
        # ═══════════════════════════════════════════════════════════════════
        # STEP 3: Morphological cleanup
        # ═══════════════════════════════════════════════════════════════════
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, 
                                        self.kernel_small, iterations=1)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, 
                                        self.kernel_large, iterations=1)
        
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, 
                                       self.kernel_small, iterations=1)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, 
                                       self.kernel_large, iterations=1)
        
        # ═══════════════════════════════════════════════════════════════════
        # STEP 4: Apply filters
        # ═══════════════════════════════════════════════════════════════════
        # Exclude brown/sand areas
        yellow_mask = cv2.bitwise_and(yellow_mask, cv2.bitwise_not(brown_mask))
        
        # Require yellow near green vegetation (plant context)
        green_area = np.sum(green_mask > 0)
        if green_area > 1000:
            green_dilated = cv2.dilate(green_mask, self.kernel_large, iterations=5)
            yellow_mask = cv2.bitwise_and(yellow_mask, green_dilated)
        
        # ═══════════════════════════════════════════════════════════════════
        # STEP 5: Find contours
        # ═══════════════════════════════════════════════════════════════════
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, 
                                        cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            
            # Filter by minimum area
            if area < self.min_area:
                continue
            
            x, y, w_box, h_box = cv2.boundingRect(cnt)
            cx, cy = x + w_box // 2, y + h_box // 2
            
            # ───────────────────────────────────────────────────────────────
            # VALIDATION 1: Green vegetation nearby
            # ───────────────────────────────────────────────────────────────
            margin = 30
            y1, y2 = max(0, y - margin), min(h, y + h_box + margin)
            x1, x2 = max(0, x - margin), min(w, x + w_box + margin)
            
            green_nearby = np.sum(green_mask[y1:y2, x1:x2] > 0)
            
            if green_area > 2000 and green_nearby < self.min_green:
                continue
            
            # ───────────────────────────────────────────────────────────────
            # VALIDATION 2: Not mostly sand/brown
            # ───────────────────────────────────────────────────────────────
            sand_nearby = np.sum(brown_mask[y1:y2, x1:x2] > 0)
            if sand_nearby > green_nearby * 1.5:
                continue
            
            # ───────────────────────────────────────────────────────────────
            # VALIDATION 3: Shape filtering
            # ───────────────────────────────────────────────────────────────
            aspect_ratio = float(w_box) / h_box if h_box > 0 else 0
            perimeter = cv2.arcLength(cnt, True)
            compactness = (4 * math.pi * area) / (perimeter ** 2) if perimeter > 0 else 0
            
            if not (0.25 <= aspect_ratio <= 4.0 and compactness > 0.25):
                continue
            
            # ───────────────────────────────────────────────────────────────
            # STEP 6: Compute GPS geotag
            # ───────────────────────────────────────────────────────────────
            lat, lon = self._pixel_to_gps(cx, cy, w, h)
            
            if lat is None:
                continue
            
            # ───────────────────────────────────────────────────────────────
            # VALIDATION 4: GPS deduplication
            # ───────────────────────────────────────────────────────────────
            if self._is_duplicate(lat, lon):
                continue
            
            self.logged_locations.append((lat, lon))
            if len(self.logged_locations) > 500:
                self.logged_locations.pop(0)
            
            # ───────────────────────────────────────────────────────────────
            # Classify severity
            # ───────────────────────────────────────────────────────────────
            if area > 1000:
                severity, score = 'SEVERE', 3
            elif area > 500:
                severity, score = 'MODERATE', 2
            else:
                severity, score = 'MILD', 1
            
            confidence = min(1.0, area / (self.min_area * 10))
            
            detections.append(Detection(
                latitude=lat, longitude=lon,
                pixel_x=cx, pixel_y=cy,
                area=int(area), severity=severity,
                severity_score=score, confidence=confidence
            ))
            
            self.get_logger().info(
                f'Disease detected: ({lat:.6f}, {lon:.6f}) - {severity}'
            )
        
        return detections
    
    def _pixel_to_gps(self, px: int, py: int, 
                      frame_w: int, frame_h: int) -> Tuple[Optional[float], Optional[float]]:
        """
        Convert pixel coordinates to GPS using ray-casting.
        
        Uses IMU orientation and camera intrinsics for accurate projection.
        Falls back to simple FOV-based calculation if scipy unavailable.
        """
        if self.current_gps is None:
            return None, None
        
        altitude = max(1.0, self.current_altitude)
        
        # Method 1: Full ray-casting with IMU (if scipy available)
        if HAS_SCIPY and self.current_imu is not None and self.R_body_cam is not None:
            # Pixel to normalized camera ray
            X_cam = (px - self.cx) / self.fx
            Y_cam = (py - self.cy) / self.fy
            Z_cam = 1.0
            
            ray_cam = np.array([X_cam, Y_cam, Z_cam])
            ray_cam = ray_cam / np.linalg.norm(ray_cam)
            
            # Camera -> Body frame
            ray_body = self.R_body_cam @ ray_cam
            
            # Body -> World frame using IMU quaternion
            q = self.current_imu.orientation
            rot = R.from_quat([q.x, q.y, q.z, q.w])
            R_world_body = rot.as_matrix()
            ray_world = R_world_body @ ray_body
            
            # Ray-ground intersection
            if ray_world[2] >= 0:
                return None, None
            
            t = altitude / (-ray_world[2])
            x_ground = ray_world[0] * t
            y_ground = ray_world[1] * t
            
            # ENU offset to GPS
            lat, lon = self._enu_to_gps(x_ground, y_ground)
            return lat, lon
        
        # Method 2: Simple FOV-based projection (fallback)
        else:
            dx = (px - frame_w / 2) / (frame_w / 2) * (self.hfov / 2)
            dy = (py - frame_h / 2) / (frame_h / 2) * (self.vfov / 2)
            
            forward = altitude * math.tan(dy)
            right = altitude * math.tan(dx)
            
            lat, lon = self._enu_to_gps(right, forward)
            return lat, lon
    
    def _enu_to_gps(self, x_offset: float, y_offset: float) -> Tuple[float, float]:
        """Convert ENU offset (meters) to GPS coordinates."""
        R_earth = 6378137.0
        
        ref_lat = self.current_gps.latitude
        ref_lon = self.current_gps.longitude
        
        dlat = (y_offset / R_earth) * (180.0 / math.pi)
        dlon = (x_offset / (R_earth * math.cos(math.radians(ref_lat)))) * (180.0 / math.pi)
        
        return ref_lat + dlat, ref_lon + dlon
    
    def _is_duplicate(self, lat: float, lon: float) -> bool:
        """Check if location is within dedup distance of previous detection."""
        for prev_lat, prev_lon in self.logged_locations:
            dist = self._haversine(lat, lon, prev_lat, prev_lon)
            if dist < self.dedup_distance:
                return True
        return False
    
    def _haversine(self, lat1: float, lon1: float, 
                   lat2: float, lon2: float) -> float:
        """Calculate distance between two GPS points in meters."""
        R = 6371000
        lat1, lon1 = math.radians(lat1), math.radians(lon1)
        lat2, lon2 = math.radians(lat2), math.radians(lon2)
        
        dlat, dlon = lat2 - lat1, lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        
        return R * 2 * math.asin(math.sqrt(a))
    
    def _publish_geotag(self, det: Detection):
        """Publish GeoPointStamped message."""
        msg = GeoPointStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.position = GeoPoint()
        msg.position.latitude = det.latitude
        msg.position.longitude = det.longitude
        msg.position.altitude = 0.0  # Ground level
        
        self.geotag_pub.publish(msg)
        self.detection_counter += 1
    
    def _log_to_csv(self, det: Detection):
        """Log detection to CSV file."""
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                f'DET{self.detection_counter:05d}',
                f'{det.latitude:.8f}',
                f'{det.longitude:.8f}',
                f'{self.current_altitude:.2f}',
                det.severity,
                det.severity_score,
                det.area,
                det.pixel_x,
                det.pixel_y,
                'DETECTED'
            ])
    
    def _draw_detections(self, frame: np.ndarray, 
                         detections: List[Detection]) -> np.ndarray:
        """Draw red highlights on detected areas."""
        debug_frame = frame.copy()
        h, w = frame.shape[:2]
        
        for det in detections:
            cx, cy = det.pixel_x, det.pixel_y
            
            # Red circle
            cv2.circle(debug_frame, (cx, cy), 15, (0, 0, 255), 3)
            
            # Red crosshair
            cv2.line(debug_frame, (cx-20, cy), (cx+20, cy), (0, 0, 255), 2)
            cv2.line(debug_frame, (cx, cy-20), (cx, cy+20), (0, 0, 255), 2)
            
            # GPS text
            gps_text = f'{det.latitude:.6f},{det.longitude:.6f}'
            cv2.putText(debug_frame, gps_text, (cx+20, cy-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # Severity
            cv2.putText(debug_frame, f'{det.severity} ({det.area}px)', 
                       (cx+20, cy+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                       (0, 0, 255), 2)
        
        # Stats overlay
        cv2.rectangle(debug_frame, (0, 0), (350, 100), (50, 50, 50), -1)
        cv2.putText(debug_frame, f'Altitude: {self.current_altitude:.1f}m',
                   (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(debug_frame, f'Detections: {len(detections)}',
                   (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(debug_frame, f'Total logged: {self.detection_counter}',
                   (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return debug_frame
    
    def _publish_debug_image(self, frame: np.ndarray):
        """Publish debug visualization."""
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            self.debug_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Debug image error: {e}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = DetectionAndGeotagNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'Total detections: {node.detection_counter}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
