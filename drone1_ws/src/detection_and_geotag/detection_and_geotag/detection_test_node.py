#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════
DISEASE DETECTION TEST NODE - Calibration & Testing Tool
═══════════════════════════════════════════════════════════════════════════════

Test node for yellow disease detection with easy HSV calibration.
Supports both simulation mode (mock GPS + laptop webcam) and real mode (MAVROS GPS + Pi Camera).

CAMERA BEHAVIOR:
    use_sim:=true  → Uses laptop webcam (/dev/video0) via OpenCV directly
    use_sim:=false → Subscribes to /camera/image_raw (Pi Camera 3 Wide on Pi 5)

HEADLESS-FRIENDLY: Works without display. Calibrate via ROS2 parameters.
When display available: Shows live preview with optional trackbars.

USAGE:
    # Simulation mode (laptop webcam + mock GPS)
    ros2 launch detection_and_geotag detection_test.launch.py use_sim:=true
    
    # Real mode (Pi Camera + Orange Cube+ GPS via MAVROS)
    ros2 launch detection_and_geotag detection_test.launch.py use_sim:=false

    # Adjust HSV via parameters (headless)
    ros2 param set /detection_test_node yellow_h_min 20
    ros2 param set /detection_test_node yellow_h_max 45

═══════════════════════════════════════════════════════════════════════════════
"""

import math
import time
import threading
from typing import Optional, Tuple, List
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Header
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPointStamped, GeoPoint

from cv_bridge import CvBridge
import cv2
import numpy as np


@dataclass
class Detection:
    """Simple detection result."""
    latitude: float
    longitude: float
    pixel_x: int
    pixel_y: int
    area: int


class DetectionTestNode(Node):
    """
    Test node for disease detection with HSV calibration.
    
    Supports:
    - use_sim=true: Laptop webcam + Mock GPS (no external nodes needed)
    - use_sim=false: Pi Camera topic + Real GPS from MAVROS
    - Live parameter updates for HSV calibration
    - Optional GUI preview when display available
    """
    
    def __init__(self):
        super().__init__('detection_test_node')
        
        # ═══════════════════════════════════════════════════════════════════
        # PARAMETERS
        # ═══════════════════════════════════════════════════════════════════
        
        # Simulation mode
        self.declare_parameter('use_sim', True)
        
        # Camera settings
        self.declare_parameter('sim_camera_device', 0)  # Laptop webcam (usually 0)
        self.declare_parameter('sim_camera_width', 640)
        self.declare_parameter('sim_camera_height', 480)
        self.declare_parameter('sim_camera_fps', 30.0)
        
        # Real mode camera topic (Pi Camera 3 Wide publishes here)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        
        # Mock GPS settings (only used when use_sim=true)
        self.declare_parameter('mock_latitude', 10.0478)
        self.declare_parameter('mock_longitude', 76.3303)
        self.declare_parameter('mock_altitude', 15.0)
        self.declare_parameter('mock_drift_rate', 0.00001)  # GPS drift per detection
        
        # Yellow detection HSV (live adjustable)
        # Full yellow range: light to dark
        self.declare_parameter('yellow_h_min', 20)   # Hue min
        self.declare_parameter('yellow_h_max', 35)   # Hue max
        self.declare_parameter('yellow_s_min', 50)   # Low for pale yellows
        self.declare_parameter('yellow_s_max', 255)
        self.declare_parameter('yellow_v_min', 50)   # Low for dark yellows
        self.declare_parameter('yellow_v_max', 255)
        
        # Detection settings
        self.declare_parameter('min_detection_area', 250)
        self.declare_parameter('gps_dedup_distance_m', 3.0)
        
        # Display settings
        self.declare_parameter('show_gui', True)  # Set false for headless
        self.declare_parameter('show_trackbars', True)  # Trackbars in GUI
        
        # ═══════════════════════════════════════════════════════════════════
        # GET PARAMETERS
        # ═══════════════════════════════════════════════════════════════════
        self.use_sim = self.get_parameter('use_sim').value
        
        # Camera settings
        self.sim_camera_device = self.get_parameter('sim_camera_device').value
        self.sim_camera_width = self.get_parameter('sim_camera_width').value
        self.sim_camera_height = self.get_parameter('sim_camera_height').value
        self.sim_camera_fps = self.get_parameter('sim_camera_fps').value
        self.camera_topic = self.get_parameter('camera_topic').value
        
        # GPS settings
        self.mock_lat = self.get_parameter('mock_latitude').value
        self.mock_lon = self.get_parameter('mock_longitude').value
        self.mock_alt = self.get_parameter('mock_altitude').value
        self.mock_drift = self.get_parameter('mock_drift_rate').value
        
        self._update_hsv_from_params()
        
        self.min_area = self.get_parameter('min_detection_area').value
        self.dedup_distance = self.get_parameter('gps_dedup_distance_m').value
        self.show_gui = self.get_parameter('show_gui').value
        self.show_trackbars = self.get_parameter('show_trackbars').value
        
        # ═══════════════════════════════════════════════════════════════════
        # STATE
        # ═══════════════════════════════════════════════════════════════════
        self.current_gps: Optional[NavSatFix] = None
        self.current_altitude: float = self.mock_alt
        self.logged_locations: List[Tuple[float, float]] = []
        self.detection_count = 0
        self.frame_count = 0
        self.gui_initialized = False
        self.camera_running = False
        self.cap = None  # OpenCV VideoCapture (simulation mode only)
        
        # Morphological kernels
        self.kernel_small = np.ones((3, 3), np.uint8)
        self.kernel_large = np.ones((5, 5), np.uint8)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # ═══════════════════════════════════════════════════════════════════
        # PARAMETER CALLBACK (for live HSV updates)
        # ═══════════════════════════════════════════════════════════════════
        self.add_on_set_parameters_callback(self._param_callback)
        
        # ═══════════════════════════════════════════════════════════════════
        # QOS
        # ═══════════════════════════════════════════════════════════════════
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # ═══════════════════════════════════════════════════════════════════
        # CAMERA SETUP (different for sim vs real)
        # ═══════════════════════════════════════════════════════════════════
        if self.use_sim:
            # SIMULATION: Use laptop webcam via OpenCV directly
            self._init_sim_camera()
            self._init_mock_gps()
            self.get_logger().info('🔧 SIMULATION MODE:')
            self.get_logger().info(f'   📷 Camera: Laptop webcam (device {self.sim_camera_device})')
            self.get_logger().info(f'   🛰️  GPS: Mock data ({self.mock_lat}, {self.mock_lon})')
        else:
            # REAL: Subscribe to Pi Camera topic + MAVROS GPS
            self.image_sub = self.create_subscription(
                Image, self.camera_topic,
                self.image_callback, sensor_qos
            )
            self.gps_sub = self.create_subscription(
                NavSatFix, '/mavros/global_position/global',
                self.gps_callback, sensor_qos
            )
            self.pose_sub = self.create_subscription(
                PoseStamped, '/mavros/local_position/pose',
                self.pose_callback, sensor_qos
            )
            self.get_logger().info('🛰️  REAL MODE:')
            self.get_logger().info(f'   📷 Camera: Pi Camera 3 Wide ({self.camera_topic})')
            self.get_logger().info('   🛰️  GPS: Orange Cube+ via MAVROS')
        
        # ═══════════════════════════════════════════════════════════════════
        # PUBLISHERS
        # ═══════════════════════════════════════════════════════════════════
        self.geotag_pub = self.create_publisher(
            GeoPointStamped, '/drone1/disease_geotag', reliable_qos
        )
        
        # ═══════════════════════════════════════════════════════════════════
        # GUI INITIALIZATION (if display available)
        # ═══════════════════════════════════════════════════════════════════
        if self.show_gui:
            self._try_init_gui()
        
        # ═══════════════════════════════════════════════════════════════════
        # LOG CONFIG
        # ═══════════════════════════════════════════════════════════════════
        self.get_logger().info('=' * 60)
        self.get_logger().info('DISEASE DETECTION TEST NODE')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Yellow HSV: H[{self.yellow_h_min}-{self.yellow_h_max}] '
                               f'S[{self.yellow_s_min}-{self.yellow_s_max}] '
                               f'V[{self.yellow_v_min}-{self.yellow_v_max}]')
        self.get_logger().info(f'Min area: {self.min_area}px')
        self.get_logger().info(f'GUI: {"Enabled" if self.gui_initialized else "Disabled (headless)"}')
        self.get_logger().info('=' * 60)
        self.get_logger().info('📝 Adjust HSV via: ros2 param set /detection_test_node yellow_h_min <value>')
        self.get_logger().info('=' * 60)
    
    def _init_sim_camera(self):
        """Initialize laptop webcam for simulation mode."""
        try:
            self.cap = cv2.VideoCapture(self.sim_camera_device)
            if not self.cap.isOpened():
                self.get_logger().error(f'❌ Failed to open camera device {self.sim_camera_device}')
                self.get_logger().error('   Try: ls /dev/video* to find available cameras')
                return
            
            # Configure camera
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.sim_camera_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.sim_camera_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.sim_camera_fps)
            
            self.camera_running = True
            
            # Start camera capture timer
            timer_period = 1.0 / self.sim_camera_fps
            self.camera_timer = self.create_timer(timer_period, self._sim_camera_callback)
            
            self.get_logger().info(f'✅ Webcam opened: {self.sim_camera_width}x{self.sim_camera_height} @ {self.sim_camera_fps}fps')
        except Exception as e:
            self.get_logger().error(f'❌ Camera error: {e}')
    
    def _sim_camera_callback(self):
        """Capture frame from laptop webcam (simulation mode)."""
        if not self.camera_running or self.cap is None:
            return
        
        ret, frame = self.cap.read()
        if not ret:
            return
        
        self._process_frame(frame)
    
    def _update_hsv_from_params(self):
        """Update HSV values from parameters."""
        self.yellow_h_min = self.get_parameter('yellow_h_min').value
        self.yellow_h_max = self.get_parameter('yellow_h_max').value
        self.yellow_s_min = self.get_parameter('yellow_s_min').value
        self.yellow_s_max = self.get_parameter('yellow_s_max').value
        self.yellow_v_min = self.get_parameter('yellow_v_min').value
        self.yellow_v_max = self.get_parameter('yellow_v_max').value
        
        self.yellow_lower = np.array([self.yellow_h_min, self.yellow_s_min, self.yellow_v_min])
        self.yellow_upper = np.array([self.yellow_h_max, self.yellow_s_max, self.yellow_v_max])
    
    def _param_callback(self, params) -> SetParametersResult:
        """Handle live parameter updates."""
        for param in params:
            if param.name.startswith('yellow_'):
                self._update_hsv_from_params()
                self.get_logger().info(
                    f'🎨 HSV updated: H[{self.yellow_h_min}-{self.yellow_h_max}] '
                    f'S[{self.yellow_s_min}-{self.yellow_s_max}] '
                    f'V[{self.yellow_v_min}-{self.yellow_v_max}]'
                )
                break
        return SetParametersResult(successful=True)
    
    def _init_mock_gps(self):
        """Initialize mock GPS for simulation mode."""
        self.current_gps = NavSatFix()
        self.current_gps.latitude = self.mock_lat
        self.current_gps.longitude = self.mock_lon
        self.current_gps.altitude = self.mock_alt
        self.current_altitude = self.mock_alt
    
    def _try_init_gui(self):
        """Try to initialize GUI windows. Fails gracefully if no display."""
        try:
            cv2.namedWindow('Detection Test', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Detection Test', 800, 600)
            
            if self.show_trackbars:
                # Create trackbars for HSV calibration
                cv2.createTrackbar('H Min', 'Detection Test', self.yellow_h_min, 179, self._on_trackbar)
                cv2.createTrackbar('H Max', 'Detection Test', self.yellow_h_max, 179, self._on_trackbar)
                cv2.createTrackbar('S Min', 'Detection Test', self.yellow_s_min, 255, self._on_trackbar)
                cv2.createTrackbar('S Max', 'Detection Test', self.yellow_s_max, 255, self._on_trackbar)
                cv2.createTrackbar('V Min', 'Detection Test', self.yellow_v_min, 255, self._on_trackbar)
                cv2.createTrackbar('V Max', 'Detection Test', self.yellow_v_max, 255, self._on_trackbar)
            
            self.gui_initialized = True
            self.get_logger().info('✅ GUI initialized successfully')
        except Exception as e:
            self.gui_initialized = False
            self.get_logger().warn(f'⚠️  No display available, running headless: {e}')
    
    def _on_trackbar(self, _):
        """Handle trackbar changes - update HSV values."""
        if not self.gui_initialized:
            return
        
        try:
            self.yellow_h_min = cv2.getTrackbarPos('H Min', 'Detection Test')
            self.yellow_h_max = cv2.getTrackbarPos('H Max', 'Detection Test')
            self.yellow_s_min = cv2.getTrackbarPos('S Min', 'Detection Test')
            self.yellow_s_max = cv2.getTrackbarPos('S Max', 'Detection Test')
            self.yellow_v_min = cv2.getTrackbarPos('V Min', 'Detection Test')
            self.yellow_v_max = cv2.getTrackbarPos('V Max', 'Detection Test')
            
            self.yellow_lower = np.array([self.yellow_h_min, self.yellow_s_min, self.yellow_v_min])
            self.yellow_upper = np.array([self.yellow_h_max, self.yellow_s_max, self.yellow_v_max])
        except Exception:
            pass
    
    def gps_callback(self, msg: NavSatFix):
        """Update GPS from MAVROS (real mode only)."""
        self.current_gps = msg
    
    def pose_callback(self, msg: PoseStamped):
        """Update altitude from MAVROS (real mode only)."""
        self.current_altitude = msg.pose.position.z
    
    def image_callback(self, msg: Image):
        """Process camera frame from ROS topic (real mode - Pi Camera)."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self._process_frame(frame)
        except Exception as e:
            self.get_logger().error(f'Frame conversion error: {e}')
    
    def _process_frame(self, frame: np.ndarray):
        """Process a frame for yellow detection (shared by sim and real mode)."""
        self.frame_count += 1
        
        # Need GPS data to proceed
        if self.current_gps is None:
            if self.frame_count % 30 == 0:
                self.get_logger().warn('Waiting for GPS data...')
            return
        
        try:
            h, w = frame.shape[:2]
            
            # Detect yellow regions
            detections = self._detect_yellow(frame)
            
            # Log and publish detections
            for det in detections:
                self._publish_geotag(det)
                self.detection_count += 1
                
                # Console output - easy to copy GPS coords
                self.get_logger().info(
                    f'🎯 DETECTION #{self.detection_count}: '
                    f'({det.latitude:.6f}, {det.longitude:.6f}) | '
                    f'Area: {det.area}px | Pixel: ({det.pixel_x}, {det.pixel_y})'
                )
                
                # Drift mock GPS for next detection (simulation mode)
                if self.use_sim:
                    self.current_gps.latitude += self.mock_drift
                    self.current_gps.longitude += self.mock_drift * 0.5
            
            # Update GUI if available
            if self.gui_initialized:
                self._update_gui(frame, detections)
                cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f'Frame processing error: {e}')
    
    def _detect_yellow(self, frame: np.ndarray) -> List[Detection]:
        """Detect yellow regions and create geotags."""
        h, w = frame.shape[:2]
        detections = []
        
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create yellow mask
        yellow_mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        
        # Morphological cleanup
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, self.kernel_small)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, self.kernel_large)
        
        # Find contours
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            
            # Filter by minimum area
            if area < self.min_area:
                continue
            
            x, y, w_box, h_box = cv2.boundingRect(cnt)
            cx, cy = x + w_box // 2, y + h_box // 2
            
            # Compute GPS geotag
            lat, lon = self._pixel_to_gps(cx, cy, w, h)
            
            # Check for duplicates
            if self._is_duplicate(lat, lon):
                continue
            
            self.logged_locations.append((lat, lon))
            if len(self.logged_locations) > 100:
                self.logged_locations.pop(0)
            
            detections.append(Detection(
                latitude=lat, longitude=lon,
                pixel_x=cx, pixel_y=cy, area=int(area)
            ))
        
        return detections
    
    def _pixel_to_gps(self, px: int, py: int, 
                      frame_w: int, frame_h: int) -> Tuple[float, float]:
        """Convert pixel to GPS using simple projection."""
        # Simple FOV-based projection (no IMU needed for testing)
        hfov = math.radians(60.0)
        vfov = math.radians(45.0)
        
        altitude = max(1.0, self.current_altitude)
        
        dx = (px - frame_w / 2) / (frame_w / 2) * (hfov / 2)
        dy = (py - frame_h / 2) / (frame_h / 2) * (vfov / 2)
        
        forward = altitude * math.tan(dy)
        right = altitude * math.tan(dx)
        
        # ENU to GPS
        R_earth = 6378137.0
        ref_lat = self.current_gps.latitude
        ref_lon = self.current_gps.longitude
        
        dlat = (forward / R_earth) * (180.0 / math.pi)
        dlon = (right / (R_earth * math.cos(math.radians(ref_lat)))) * (180.0 / math.pi)
        
        return ref_lat + dlat, ref_lon + dlon
    
    def _is_duplicate(self, lat: float, lon: float) -> bool:
        """Check if location is duplicate."""
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
        msg.position.altitude = 0.0
        
        self.geotag_pub.publish(msg)
    
    def _update_gui(self, frame: np.ndarray, detections: List[Detection]):
        """Update GUI with preview and mask overlay."""
        h, w = frame.shape[:2]
        
        # Create yellow mask for display
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        
        # Create side-by-side display
        mask_colored = cv2.cvtColor(yellow_mask, cv2.COLOR_GRAY2BGR)
        
        # Draw detections on frame
        for det in detections:
            cv2.circle(frame, (det.pixel_x, det.pixel_y), 15, (0, 255, 0), 3)
            cv2.putText(frame, f'{det.latitude:.4f},{det.longitude:.4f}',
                       (det.pixel_x + 20, det.pixel_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Status overlay
        mode_text = 'SIM' if self.use_sim else 'REAL'
        cv2.rectangle(frame, (0, 0), (400, 90), (50, 50, 50), -1)
        cv2.putText(frame, f'Mode: {mode_text} | Detections: {self.detection_count}',
                   (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f'GPS: {self.current_gps.latitude:.6f}, {self.current_gps.longitude:.6f}',
                   (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(frame, f'HSV: H[{self.yellow_h_min}-{self.yellow_h_max}] '
                          f'S[{self.yellow_s_min}-{self.yellow_s_max}] '
                          f'V[{self.yellow_v_min}-{self.yellow_v_max}]',
                   (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        # Combine frame and mask
        display = np.hstack([frame, mask_colored])
        cv2.imshow('Detection Test', display)
    
    def destroy_node(self):
        """Clean up camera and GUI on shutdown."""
        # Stop camera (simulation mode)
        self.camera_running = False
        if self.cap is not None:
            self.cap.release()
            self.get_logger().info('📷 Camera released')
        
        # Close GUI
        if self.gui_initialized:
            cv2.destroyAllWindows()
        
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = DetectionTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'Total detections: {node.detection_count}')
        
        # Print final HSV values for easy copying
        node.get_logger().info('=' * 60)
        node.get_logger().info('FINAL HSV VALUES (copy to config):')
        node.get_logger().info(f'  yellow_h_min: {node.yellow_h_min}')
        node.get_logger().info(f'  yellow_h_max: {node.yellow_h_max}')
        node.get_logger().info(f'  yellow_s_min: {node.yellow_s_min}')
        node.get_logger().info(f'  yellow_s_max: {node.yellow_s_max}')
        node.get_logger().info(f'  yellow_v_min: {node.yellow_v_min}')
        node.get_logger().info(f'  yellow_v_max: {node.yellow_v_max}')
        node.get_logger().info('=' * 60)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
