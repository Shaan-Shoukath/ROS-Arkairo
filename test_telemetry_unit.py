#!/usr/bin/env python3
"""
Telemetry Unit Test (No MAVROS Required)
==========================================
Tests TX and RX nodes in isolation by simulating the MAVLink transport layer.

This script runs both TX and RX in the same process and bridges them with
a mock MAVLink transport.

Usage:
  python3 test_telemetry_unit.py
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import NamedValueFloat
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header, Bool


class MockMAVLinkBridge(Node):
    """Simulates MAVLink transport by bridging TX → RX topics."""
    
    def __init__(self):
        super().__init__('mock_mavlink_bridge')
        
        # Subscribe to TX output
        self.tx_sub = self.create_subscription(
            NamedValueFloat,
            '/mavros/named_value_float/send',
            self.forward_to_rx,
            10
        )
        
        # Publish to RX input
        self.rx_pub = self.create_publisher(
            NamedValueFloat,
            '/mavros/named_value_float',
            10
        )
        
        self.forwarded_count = 0
        self.get_logger().info('Mock MAVLink Bridge initialized')
    
    def forward_to_rx(self, msg: NamedValueFloat):
        """Forward TX messages to RX (simulates MAVLink transport)."""
        # Add small delay to simulate network latency
        time.sleep(0.01)
        
        self.rx_pub.publish(msg)
        self.forwarded_count += 1
        
        self.get_logger().debug(
            f'Forwarded: {msg.name}={msg.value:.6f} (count={self.forwarded_count})'
        )


class TestHarness(Node):
    """Test controller: sends test data and validates results."""
    
    def __init__(self):
        super().__init__('test_harness')
        
        # Publisher: send test geotags to TX
        self.geotag_pub = self.create_publisher(
            GeoPointStamped,
            '/drone1/disease_geotag',
            10
        )
        
        # Subscribers: monitor RX output
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
        self.sent_tests = []
        self.received_targets = []
        self.triggers = 0
        
        self.get_logger().info('Test Harness initialized')
    
    def target_callback(self, msg: NavSatFix):
        """Capture RX output."""
        self.received_targets.append((msg.latitude, msg.longitude, msg.altitude))
        self.get_logger().info(
            f'✓ RX Output: ({msg.latitude:.6f}, {msg.longitude:.6f}, {msg.altitude:.1f}m)'
        )
    
    def trigger_callback(self, msg: Bool):
        """Count triggers."""
        if msg.data:
            self.triggers += 1
    
    def send_geotag(self, lat: float, lon: float, alt: float):
        """Send test geotag to TX."""
        msg = GeoPointStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.position.latitude = lat
        msg.position.longitude = lon
        msg.position.altitude = alt
        
        self.geotag_pub.publish(msg)
        self.sent_tests.append((lat, lon, alt))
        
        self.get_logger().info(f'→ TX Input: ({lat:.6f}, {lon:.6f}, {alt:.1f}m)')


def main():
    """Run unit tests."""
    rclpy.init()
    
    # Create all nodes
    bridge = MockMAVLinkBridge()
    harness = TestHarness()
    
    # Import actual TX and RX nodes
    try:
        from telem_tx.telem_tx_node import TelemTxNode
        from telem_rx.telem_rx_node import TelemRxNode
    except ImportError:
        print("ERROR: Cannot import telem_tx or telem_rx nodes.")
        print("Make sure to source the workspace:")
        print("  cd ~/Documents/ROSArkairo/drone1_ws && source install/setup.zsh")
        print("  cd ~/Documents/ROSArkairo/drone2_ws && source install/setup.zsh")
        return
    
    tx_node = TelemTxNode()
    rx_node = TelemRxNode()
    
    # Override RX parameters for testing
    rx_node.home_lat = 10.0483
    rx_node.home_lon = 76.3310
    rx_node.target_alt = 10.0
    rx_node.override_alt = True
    
    # Use multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    executor.add_node(harness)
    executor.add_node(tx_node)
    executor.add_node(rx_node)
    
    print("\n" + "=" * 60)
    print("TELEMETRY UNIT TEST")
    print("=" * 60)
    print("Testing: TX → Mock MAVLink → RX")
    print("Waiting for nodes to initialize...")
    time.sleep(1.0)
    
    # Define test cases
    test_cases = [
        (10.0483, 76.3310, 5.0, "A1 - Home"),
        (10.0485, 76.3312, 5.5, "A2 - Offset NE"),
        (10.0481, 76.3308, 6.0, "B1 - Offset SW"),
    ]
    
    # Run tests
    for i, (lat, lon, alt, label) in enumerate(test_cases, 1):
        print(f"\n--- Test {i}/{len(test_cases)}: {label} ---")
        harness.send_geotag(lat, lon, alt)
        
        # Spin to process messages
        executor.spin_once(timeout_sec=0.5)
        time.sleep(1.0)  # Allow transport and processing
        executor.spin_once(timeout_sec=0.5)
    
    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    print(f"Sent:              {len(harness.sent_tests)}")
    print(f"Received:          {len(harness.received_targets)}")
    print(f"Triggers:          {harness.triggers}")
    print(f"MAVLink forwarded: {bridge.forwarded_count}")
    
    if len(harness.received_targets) == len(harness.sent_tests):
        print("\n✓ ALL TESTS PASSED")
    else:
        print(f"\n✗ FAILED: {len(harness.sent_tests) - len(harness.received_targets)} messages lost")
    
    print("=" * 60 + "\n")
    
    # Cleanup
    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nTest interrupted.")
    except Exception as e:
        print(f"\nTest failed: {e}")
        import traceback
        traceback.print_exc()
