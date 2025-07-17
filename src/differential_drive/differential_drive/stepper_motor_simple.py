#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import threading
import time
from typing import Optional

class SimpleStepperNode(Node):
    def __init__(self):
        super().__init__('simple_stepper_node')

        # Simple parameters
        self.declare_parameter('serial_port', '/dev/tty_arduino')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('button_start', 0)  # A button
        self.declare_parameter('button_stop', 1)   # B button

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.button_start = self.get_parameter('button_start').get_parameter_value().integer_value
        self.button_stop = self.get_parameter('button_stop').get_parameter_value().integer_value

        # Serial connection
        self.serial_connection: Optional[serial.Serial] = None
        self.connection_active = False

        # Button state tracking
        self.prev_start_state = False
        self.prev_stop_state = False
        self.last_command_time = 0.0
        self.min_interval = 0.5

        # Motor state
        self.motor_enabled = False

        # Create subscriber for Joy messages
        self.joy_subscriber = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        # Try to connect to Arduino
        self.connect_to_arduino()

        self.get_logger().info('Simple Stepper Motor Node Started')
        self.get_logger().info(f'Works with simple Arduino code that runs continuously')
        self.get_logger().info(f'A button (index {self.button_start}): Enable motor')
        self.get_logger().info(f'B button (index {self.button_stop}): Disable motor')
        self.get_logger().info(f'Motor starts disabled - press A to enable')

    def connect_to_arduino(self):
        """Connect to Arduino - just for monitoring, motor runs independently"""
        try:
            self.get_logger().info(f'Connecting to Arduino at {self.serial_port}...')
            
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0
            )
            
            self.connection_active = True
            self.get_logger().info('Connected to Arduino (simple mode)')
            
        except Exception as e:
            self.get_logger().warn(f'Could not connect to Arduino: {e}')
            self.get_logger().info('Continuing in button-only mode')
            self.connection_active = False

    def joy_callback(self, msg: Joy):
        """Handle joystick input"""
        # Check if we have enough buttons
        max_button = max(self.button_start, self.button_stop)
        if len(msg.buttons) <= max_button:
            self.get_logger().warn(f'Not enough buttons. Need at least {max_button + 1}')
            return

        # Get button states
        start_pressed = msg.buttons[self.button_start] == 1
        stop_pressed = msg.buttons[self.button_stop] == 1

        current_time = time.time()
        
        # Handle A button (Enable motor)
        if (start_pressed and not self.prev_start_state and 
            current_time - self.last_command_time > self.min_interval):
            
            if not self.motor_enabled:
                self.motor_enabled = True
                self.last_command_time = current_time
                self.get_logger().info('🟢 A BUTTON: Motor ENABLED - Arduino will run continuously')
                self.get_logger().info('   (The simple Arduino code runs the motor automatically)')
            else:
                self.get_logger().info('A button pressed but motor already enabled')

        # Handle B button (Disable motor)
        if (stop_pressed and not self.prev_stop_state and 
            current_time - self.last_command_time > self.min_interval):
            
            if self.motor_enabled:
                self.motor_enabled = False
                self.last_command_time = current_time
                self.get_logger().info('🔴 B BUTTON: Motor DISABLED')
                self.get_logger().info('   (You need to power cycle Arduino to stop the motor)')
                self.get_logger().info('   (Or upload different Arduino code that can be stopped)')
            else:
                self.get_logger().info('B button pressed but motor already disabled')

        # Update previous states
        self.prev_start_state = start_pressed
        self.prev_stop_state = stop_pressed

    def get_motor_state(self) -> str:
        """Get current motor state"""
        return "ENABLED" if self.motor_enabled else "DISABLED"

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down Simple Stepper Node')
        if self.serial_connection:
            try:
                self.serial_connection.close()
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleStepperNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main() 