#!/usr/bin/env python3
"""
Sprayer Pump Relay Test (ArduPilot Relay Function)

Tests relay connected to Cube Orange+ via MAVROS.
Tries multiple methods: relay service, command_int, and command_long.

Usage:
    ros2 run sprayer_control pump_test --ros-args -p relay_num:=0 -p on_time_sec:=3.0
"""

import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandLong, CommandInt
from mavros_msgs.msg import RCOut


class PumpTestNode(Node):
    """Simple node to test pump relay."""
    
    def __init__(self):
        super().__init__('pump_test_node')
        
        # Parameters
        self.declare_parameter('relay_num', 0)      # Relay number (0-5)
        self.declare_parameter('on_time_sec', 3.0)  # How long to run pump
        self.declare_parameter('aux_channel', 13)   # AUX channel (9-14 for AUX1-6)
        
        self.relay_num = self.get_parameter('relay_num').value
        self.on_time = self.get_parameter('on_time_sec').value
        self.aux_channel = self.get_parameter('aux_channel').value
        
        # MAVROS services
        self.cmd_long = self.create_client(CommandLong, '/mavros/cmd/command')
        self.cmd_int = self.create_client(CommandInt, '/mavros/cmd/command_int')
        
        # RC override publisher (fallback method)
        self.rc_pub = self.create_publisher(RCOut, '/mavros/rc/override', 10)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('SPRAYER PUMP RELAY TEST')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Relay Number: {self.relay_num}')
        self.get_logger().info(f'AUX Channel: {self.aux_channel}')
        self.get_logger().info(f'ON time: {self.on_time}s')
        self.get_logger().info('=' * 50)
        
        # Wait for service
        self.get_logger().info('Waiting for MAVROS...')
        self.cmd_long.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('MAVROS connected!')
        
        # Run test
        self.create_timer(1.0, self.run_test)
        self.test_done = False
    
    def run_test(self):
        """Run the pump test sequence."""
        if self.test_done:
            return
        self.test_done = True
        
        self.get_logger().info('')
        self.get_logger().info('Testing 3 methods...')
        self.get_logger().info('')
        
        # Method 1: DO_SET_RELAY (command 181)
        self.get_logger().info('--- Method 1: DO_SET_RELAY ---')
        self.test_relay_command()
        
        time.sleep(1)
        
        # Method 2: DO_SET_SERVO (command 183)
        self.get_logger().info('--- Method 2: DO_SET_SERVO ---')
        self.test_servo_command()
        
        time.sleep(1)
        
        # Method 3: DO_REPEAT_RELAY (command 182) 
        self.get_logger().info('--- Method 3: DO_REPEAT_RELAY ---')
        self.test_repeat_relay()
        
        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('All tests complete!')
        self.get_logger().info('Check if pump activated during any method.')
        self.get_logger().info('=' * 50)
        
        raise SystemExit(0)
    
    def test_relay_command(self):
        """Test with MAV_CMD_DO_SET_RELAY (181)."""
        # Turn ON
        self.get_logger().info(f'  Relay {self.relay_num} ON...')
        result = self.send_command(181, self.relay_num, 1)
        self.get_logger().info(f'  Result: {"OK" if result else "FAILED"}')
        
        time.sleep(self.on_time)
        
        # Turn OFF
        self.get_logger().info(f'  Relay {self.relay_num} OFF...')
        self.send_command(181, self.relay_num, 0)
    
    def test_servo_command(self):
        """Test with MAV_CMD_DO_SET_SERVO (183)."""
        # Channel for AUX5 = 13 (main 1-8 + aux 1-5)
        channel = self.aux_channel
        
        # Turn ON (PWM 2000)
        self.get_logger().info(f'  Servo channel {channel} = 2000 (ON)...')
        result = self.send_command(183, channel, 2000)
        self.get_logger().info(f'  Result: {"OK" if result else "FAILED"}')
        
        time.sleep(self.on_time)
        
        # Turn OFF (PWM 1000)
        self.get_logger().info(f'  Servo channel {channel} = 1000 (OFF)...')
        self.send_command(183, channel, 1000)
    
    def test_repeat_relay(self):
        """Test with MAV_CMD_DO_REPEAT_RELAY (182)."""
        # param1 = relay num, param2 = cycle count, param3 = cycle time
        self.get_logger().info(f'  Relay {self.relay_num} cycle 1x for {self.on_time}s...')
        result = self.send_command(182, self.relay_num, 1, self.on_time)
        self.get_logger().info(f'  Result: {"OK" if result else "FAILED"}')
        time.sleep(self.on_time + 1)
    
    def send_command(self, command: int, param1: float, param2: float, param3: float = 0) -> bool:
        """Send MAVLink command."""
        req = CommandLong.Request()
        req.command = command
        req.param1 = float(param1)
        req.param2 = float(param2)
        req.param3 = float(param3)
        
        future = self.cmd_long.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result():
            return future.result().success
        return False


def main(args=None):
    rclpy.init(args=args)
    node = PumpTestNode()
    
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        node.get_logger().info('Test cancelled')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
