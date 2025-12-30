#!/usr/bin/env python3
"""
Telemetry Pair Test Script
===========================
Tests the complete telemetry chain:
  Drone-1 TX → MAVLink → Drone-2 RX

Simulates:
  - Disease detections from Drone-1 (dummy GPS coordinates)
  - MAVLink transport (using ROS topics)
  - Reception and validation on Drone-2

Usage:
  # Terminal 1: Run telem_tx (Drone-1)
  ros2 run telem_tx telem_tx_node

  # Terminal 2: Run telem_rx (Drone-2)
  ros2 run telem_rx telem_rx_node --ros-args \
    -p home_latitude:=10.0483 \
    -p home_longitude:=76.331 \
    -p target_altitude_m:=10.0

  # Terminal 3: Run this test script
  python3 test_telemetry_pair.py
"""

import time
import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPointStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Header


class TelemetryTester(Node):
    """Test harness for telemetry TX/RX pair."""
    
    def __init__(self):
        super().__init__('telemetry_tester')
        
        # Publishers (simulate Drone-1 detections)
        self.geotag_pub = self.create_publisher(
            GeoPointStamped,
            '/drone1/disease_geotag',
            10
        )
        
        # Subscribers (monitor Drone-2 outputs)
        self.target_sub = self.create_subscription(
            NavSatFix,
            '/drone2/target_position',
            self.target_callback,
            10
        )
        
        self.trigger_sub = self.create_subscription(
            Bool,
            '/drone2/new_target_received',
            self.trigger_callback,
            10
        )
        
        # Test state
        self.test_cases = []
        self.received_targets = []
        self.trigger_count = 0
        
        self.get_logger().info('Telemetry Tester initialized')
    
    def target_callback(self, msg: NavSatFix):
        """Capture received targets from RX."""
        self.received_targets.append({
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude,
            'time': time.time()
        })
        self.get_logger().info(
            f'RX → Target received: ({msg.latitude:.6f}, {msg.longitude:.6f}, {msg.altitude:.1f}m)'
        )
    
    def trigger_callback(self, msg: Bool):
        """Count trigger events."""
        if msg.data:
            self.trigger_count += 1
            self.get_logger().info(f'RX → Trigger #{self.trigger_count}')
    
    def send_test_geotag(self, lat: float, lon: float, alt: float, label: str = ""):
        """Send a dummy disease geotag."""
        msg = GeoPointStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.position.latitude = lat
        msg.position.longitude = lon
        msg.position.altitude = alt
        
        self.geotag_pub.publish(msg)
        self.test_cases.append({
            'lat': lat,
            'lon': lon,
            'alt': alt,
            'label': label,
            'time': time.time()
        })
        
        self.get_logger().info(
            f'TX → Sent {label}: ({lat:.6f}, {lon:.6f}, {alt:.1f}m)'
        )


def run_tests():
    """Execute telemetry test suite."""
    rclpy.init()
    
    tester = TelemetryTester()
    
    # Define test cases: Kerala region (near SITL home)
    test_data = [
        # (lat, lon, alt, label)
        (10.0483, 76.3310, 5.0, "Test A1 - Near home"),
        (10.0485, 76.3312, 5.5, "Test A2 - Offset 20m NE"),
        (10.0481, 76.3308, 6.0, "Test B1 - Offset 20m SW"),
        (10.0490, 76.3320, 7.0, "Test B2 - Offset 100m NE"),
        (10.0475, 76.3300, 8.0, "Test A3 - Offset 100m SW"),
    ]
    
    tester.get_logger().info("=" * 60)
    tester.get_logger().info("TELEMETRY PAIR TEST SUITE")
    tester.get_logger().info("=" * 60)
    tester.get_logger().info(f"Test cases: {len(test_data)}")
    tester.get_logger().info("Waiting 2 seconds for nodes to initialize...")
    time.sleep(2.0)
    
    # Send test cases with delays
    for i, (lat, lon, alt, label) in enumerate(test_data, 1):
        tester.get_logger().info(f"\n--- Test {i}/{len(test_data)} ---")
        tester.send_test_geotag(lat, lon, alt, label)
        
        # Spin to process callbacks
        rclpy.spin_once(tester, timeout_sec=0.5)
        
        # Wait for telemetry transport and processing
        time.sleep(2.0)
        rclpy.spin_once(tester, timeout_sec=0.5)
    
    # Final summary
    tester.get_logger().info("\n" + "=" * 60)
    tester.get_logger().info("TEST SUMMARY")
    tester.get_logger().info("=" * 60)
    tester.get_logger().info(f"Sent:     {len(tester.test_cases)} disease geotags")
    tester.get_logger().info(f"Received: {len(tester.received_targets)} targets")
    tester.get_logger().info(f"Triggers: {tester.trigger_count}")
    
    # Check for missed transmissions
    if len(tester.received_targets) < len(tester.test_cases):
        missed = len(tester.test_cases) - len(tester.received_targets)
        tester.get_logger().warn(f"WARNING: {missed} targets were not received!")
    elif len(tester.received_targets) == len(tester.test_cases):
        tester.get_logger().info("✓ All test cases received successfully!")
    
    # Detailed comparison
    tester.get_logger().info("\nDetailed Results:")
    for i, sent in enumerate(tester.test_cases, 1):
        tester.get_logger().info(
            f"  {i}. {sent['label']:30s} "
            f"({sent['lat']:.6f}, {sent['lon']:.6f}, {sent['alt']:.1f}m)"
        )
        
        # Try to find matching received target (within 2s window)
        matched = False
        for rx in tester.received_targets:
            if abs(rx['time'] - sent['time']) < 3.0:  # 3s tolerance
                lat_match = abs(rx['lat'] - sent['lat']) < 0.0001
                lon_match = abs(rx['lon'] - sent['lon']) < 0.0001
                if lat_match and lon_match:
                    alt_note = f"alt={rx['alt']:.1f}m" if rx['alt'] != sent['alt'] else "alt matched"
                    tester.get_logger().info(f"     → RX: MATCHED ({alt_note})")
                    matched = True
                    break
        
        if not matched:
            tester.get_logger().warn("     → RX: NOT FOUND")
    
    tester.get_logger().info("\n" + "=" * 60)
    
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        run_tests()
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    except Exception as e:
        print(f"Test failed: {e}")
        import traceback
        traceback.print_exc()
