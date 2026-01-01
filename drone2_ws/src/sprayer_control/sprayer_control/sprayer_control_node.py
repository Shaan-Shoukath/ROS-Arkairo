#!/usr/bin/env python3
# Copyright 2024 Shaan Shoukath
# SPDX-License-Identifier: Apache-2.0

"""
Sprayer Control Node

Controls spray actuation via:
  - Pi 5 GPIO (default/recommended) - Direct relay control
  - PWM topic (SITL testing)
  - MAVROS servo (FC relay)

Includes priming delay before spray activation.

State Machine:
    IDLE → spray_ready=True → PRIMING (delay) → SPRAYING → DONE → IDLE

Data Flow:
    Navigation (arrives) → Detection/Centering (centers) → spray_ready → Sprayer

Subscribers:
    /drone2/spray_ready (std_msgs/Bool): Spray activation trigger
    /mavros/state (mavros_msgs/State): Flight controller state

Publishers:
    /drone2/spray_done (std_msgs/Bool): Spray operation complete
    /drone2/pwm_spray (std_msgs/UInt16): PWM output value (sim mode only)
"""

from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, UInt16
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong

# Try to import gpiozero for Pi 5 GPIO control
try:
    from gpiozero import OutputDevice
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False


class SprayState(Enum):
    """Sprayer state machine states."""
    IDLE = auto()
    PRIMING = auto()  # Delay before spray activation
    SPRAYING = auto()


class SprayerControlNode(Node):
    """ROS2 node for spray actuation control with priming delay and hardware support."""
    
    # MAVLink command for servo control
    MAV_CMD_DO_SET_SERVO = 183
    
    def __init__(self):
        super().__init__('sprayer_control_node')
        
        # ================================================================
        # PARAMETERS
        # ================================================================
        # Control mode: 'gpio' (Pi 5), 'pwm' (simulation), 'mavros' (FC servo)
        self.declare_parameter('control_mode', 'gpio')  # Default: Pi GPIO
        
        # GPIO settings (Pi 5)
        self.declare_parameter('gpio_pin', 17)  # BCM pin number
        self.declare_parameter('gpio_active_high', True)  # True = relay activates on HIGH
        
        # PWM settings (simulation)
        self.declare_parameter('pwm_channel', 9)
        self.declare_parameter('pwm_off', 1000)
        self.declare_parameter('pwm_on', 2000)
        
        # MAVROS settings (FC relay)
        self.declare_parameter('servo_channel', 10)
        
        # Spray timing
        self.declare_parameter('spray_duration_sec', 5.0)
        self.declare_parameter('spray_start_delay_sec', 2.5)  # Priming delay
        
        # Safety
        self.declare_parameter('require_armed', True)
        self.declare_parameter('require_guided', True)
        self.declare_parameter('min_altitude_m', 2.0)
        
        # Legacy compatibility
        self.declare_parameter('use_sim', True)  # Deprecated, use control_mode instead
        
        # Get parameters
        self.control_mode = self.get_parameter('control_mode').value
        self.gpio_pin = self.get_parameter('gpio_pin').value
        self.gpio_active_high = self.get_parameter('gpio_active_high').value
        self.pwm_channel = self.get_parameter('pwm_channel').value
        self.pwm_off = self.get_parameter('pwm_off').value
        self.pwm_on = self.get_parameter('pwm_on').value
        self.servo_channel = self.get_parameter('servo_channel').value
        self.spray_duration = self.get_parameter('spray_duration_sec').value
        self.spray_start_delay = self.get_parameter('spray_start_delay_sec').value
        self.require_armed = self.get_parameter('require_armed').value
        self.require_guided = self.get_parameter('require_guided').value
        self.min_alt = self.get_parameter('min_altitude_m').value
        
        # Handle legacy use_sim parameter
        use_sim = self.get_parameter('use_sim').value
        if use_sim and self.control_mode == 'gpio':
            self.control_mode = 'pwm'  # Override to PWM for SITL compatibility
        
        # ================================================================
        # GPIO INITIALIZATION
        # ================================================================
        self.relay = None
        if self.control_mode == 'gpio':
            if HAS_GPIO:
                try:
                    self.relay = OutputDevice(
                        self.gpio_pin, 
                        active_high=self.gpio_active_high,
                        initial_value=False  # Start OFF
                    )
                    self.get_logger().info(f'GPIO initialized: pin {self.gpio_pin}')
                except Exception as e:
                    self.get_logger().error(f'GPIO init failed: {e}')
                    self.control_mode = 'pwm'  # Fallback to PWM
            else:
                self.get_logger().warn('gpiozero not available, falling back to PWM mode')
                self.control_mode = 'pwm'
        
        # ================================================================
        # STATE VARIABLES
        # ================================================================
        self.state = SprayState.IDLE
        self.fc_state: Optional[State] = None
        self.state_start_time = None
        
        # Statistics
        self.total_sprays = 0
        
        # ================================================================
        # QOS
        # ================================================================
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
        
        # ================================================================
        # SUBSCRIBERS
        # ================================================================
        self.ready_sub = self.create_subscription(
            Bool, '/drone2/spray_ready',
            self.ready_callback, reliable_qos
        )
        
        self.state_sub = self.create_subscription(
            State, '/mavros/state',
            self.state_callback, sensor_qos
        )
        
        # ================================================================
        # PUBLISHERS
        # ================================================================
        self.done_pub = self.create_publisher(
            Bool, '/drone2/spray_done', reliable_qos
        )
        
        self.pwm_pub = self.create_publisher(
            UInt16, '/drone2/pwm_spray', 10
        )
        
        # ================================================================
        # SERVICE CLIENTS (for MAVROS mode)
        # ================================================================
        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')
        
        # ================================================================
        # TIMERS
        # ================================================================
        self.spray_timer = self.create_timer(0.1, self.spray_loop)
        
        # Ensure sprayer is off at start
        self._set_spray_output(False)
        
        # ================================================================
        # LOGGING
        # ================================================================
        mode_desc = {
            'gpio': f'Pi GPIO (pin {self.gpio_pin})',
            'pwm': 'PWM topic (simulation)',
            'mavros': f'MAVROS servo (channel {self.servo_channel})'
        }
        self.get_logger().info('=' * 50)
        self.get_logger().info('Sprayer Control Node initialized')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'  Mode: {mode_desc.get(self.control_mode, self.control_mode)}')
        self.get_logger().info(f'  Priming delay: {self.spray_start_delay}s')
        self.get_logger().info(f'  Spray duration: {self.spray_duration}s')
        if self.control_mode == 'pwm':
            self.get_logger().info(f'  PWM: {self.pwm_off} (off) -> {self.pwm_on} (on)')
    
    # ====================================================================
    # CALLBACKS
    # ====================================================================
    
    def state_callback(self, msg: State):
        """Update flight controller state."""
        self.fc_state = msg
    
    def ready_callback(self, msg: Bool):
        """Handle spray ready trigger."""
        if not msg.data:
            return
        
        if self.state != SprayState.IDLE:
            self.get_logger().warn(f'Spray request ignored - currently in {self.state.name}')
            return
        
        # Safety checks
        if not self._safety_check():
            self.get_logger().error('Safety check failed - spray aborted')
            self._publish_done(False)
            return
        
        # Start priming phase
        self._transition_to(SprayState.PRIMING)
        self.get_logger().info(f'Priming delay started ({self.spray_start_delay}s)...')
    
    # ====================================================================
    # STATE MACHINE
    # ====================================================================
    
    def _transition_to(self, new_state: SprayState):
        """Transition to a new state."""
        old_state = self.state
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        self.get_logger().debug(f'State: {old_state.name} -> {new_state.name}')
    
    def _time_in_state(self) -> float:
        """Get time elapsed in current state (seconds)."""
        if self.state_start_time is None:
            return 0.0
        return (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
    
    def spray_loop(self):
        """Main state machine loop (10Hz)."""
        
        if self.state == SprayState.IDLE:
            # Nothing to do
            return
        
        elif self.state == SprayState.PRIMING:
            # Wait for priming delay
            elapsed = self._time_in_state()
            
            if elapsed >= self.spray_start_delay:
                # Priming complete - start spraying
                self._set_spray_output(True)
                self.total_sprays += 1
                self._transition_to(SprayState.SPRAYING)
                self.get_logger().info('Spraying started')
        
        elif self.state == SprayState.SPRAYING:
            # Monitor spray duration
            elapsed = self._time_in_state()
            
            if elapsed >= self.spray_duration:
                # Spray complete
                self._stop_spray()
                self._publish_done(True)
                total_time = self.spray_start_delay + self.spray_duration
                self.get_logger().info(
                    f'Spraying complete (priming: {self.spray_start_delay}s + '
                    f'spray: {elapsed:.1f}s = {total_time:.1f}s total)'
                )
    
    # ====================================================================
    # SPRAY OUTPUT CONTROL
    # ====================================================================
    
    def _set_spray_output(self, on: bool):
        """
        Set spray output based on control mode.
        
        Modes:
            - gpio: Direct Pi 5 GPIO relay control (fastest, recommended)
            - pwm: Publish to PWM topic (SITL testing)
            - mavros: MAVROS servo command (FC relay)
        
        Args:
            on: True to activate spray, False to deactivate
        """
        if self.control_mode == 'gpio':
            self._set_gpio(on)
        elif self.control_mode == 'pwm':
            pwm_value = self.pwm_on if on else self.pwm_off
            self._set_pwm_topic(pwm_value)
        elif self.control_mode == 'mavros':
            pwm_value = self.pwm_on if on else self.pwm_off
            self._set_servo_command(pwm_value)
    
    def _set_gpio(self, on: bool):
        """Control relay via Pi 5 GPIO."""
        if self.relay is None:
            self.get_logger().error('GPIO not initialized')
            return
        
        if on:
            self.relay.on()
            self.get_logger().debug(f'GPIO {self.gpio_pin}: ON')
        else:
            self.relay.off()
            self.get_logger().debug(f'GPIO {self.gpio_pin}: OFF')
    
    def _set_pwm_topic(self, value: int):
        """Publish PWM value to topic (simulation mode)."""
        msg = UInt16()
        msg.data = value
        self.pwm_pub.publish(msg)
        self.get_logger().debug(f'PWM topic: {value}')
    
    def _set_servo_command(self, pwm_value: int):
        """
        Send MAV_CMD_DO_SET_SERVO via MAVROS (hardware mode).
        
        ArduPilot servo function: The servo_channel parameter should correspond
        to a servo configured with SERVOx_FUNCTION = 0 (passthrough).
        """
        if not self.command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('MAVROS command service not available')
            return
        
        request = CommandLong.Request()
        request.broadcast = False
        request.command = self.MAV_CMD_DO_SET_SERVO
        request.confirmation = 0
        request.param1 = float(self.servo_channel)  # Servo instance number
        request.param2 = float(pwm_value)           # PWM value (1000-2000)
        request.param3 = 0.0
        request.param4 = 0.0
        request.param5 = 0.0
        request.param6 = 0.0
        request.param7 = 0.0
        
        future = self.command_client.call_async(request)
        future.add_done_callback(self._servo_command_callback)
        
        self.get_logger().info(f'Servo command: channel={self.servo_channel}, PWM={pwm_value}')
    
    def _servo_command_callback(self, future):
        """Handle servo command response."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().debug('Servo command accepted')
            else:
                self.get_logger().warn(f'Servo command failed: result={response.result}')
        except Exception as e:
            self.get_logger().error(f'Servo command error: {e}')
    
    # ====================================================================
    # SAFETY & HELPERS
    # ====================================================================
    
    def _safety_check(self) -> bool:
        """Perform safety checks before spraying."""
        if self.fc_state is None:
            self.get_logger().warn('No FC state available')
            return False
        
        if self.require_armed and not self.fc_state.armed:
            self.get_logger().warn('Vehicle not armed')
            return False
        
        if self.require_guided and self.fc_state.mode != 'GUIDED':
            self.get_logger().warn(f'Not in GUIDED mode: {self.fc_state.mode}')
            return False
        
        return True
    
    def _stop_spray(self):
        """Stop spraying and return to IDLE."""
        self._set_spray_output(False)
        self._transition_to(SprayState.IDLE)
    
    def _publish_done(self, success: bool):
        """Publish spray done status."""
        msg = Bool()
        msg.data = success
        self.done_pub.publish(msg)
    
    def destroy_node(self):
        """Ensure sprayer is off and clean up GPIO on shutdown."""
        self._set_spray_output(False)
        
        # Close GPIO relay if initialized
        if self.relay is not None:
            self.relay.close()
            self.get_logger().debug('GPIO relay closed')
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SprayerControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'Total sprays: {node.total_sprays}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
