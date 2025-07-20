#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import serial
import time
import threading
from typing import Optional

class StepperSimpleNode(Node):
    def __init__(self):
        super().__init__('stepper_simple_node')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/tty_arduino')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('button_start', 0)  # A button
        self.declare_parameter('button_stop', 1)   # B button
        self.declare_parameter('button_test', 2)   # X button for testing
        
        # Create publishers
        self.status_pub = self.create_publisher(String, 'stepper/status', 10)
        
        # Create Joy subscriber
        self.joy_subscriber = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        # Button state tracking
        self.stepper_running = False
        self.prev_start_state = False
        self.prev_stop_state = False
        self.prev_test_state = False

        # Initialize serial connection
        self.serial_connection: Optional[serial.Serial] = None
        self.serial_lock = threading.Lock()
        
        try:
            port = self.get_parameter('port').value
            baud = self.get_parameter('baud_rate').value
            self.serial_connection = serial.Serial(port, int(baud), timeout=1.0)
            time.sleep(2)  # Wait for Arduino to reset
            self.get_logger().info(f"🔌 Connected to Arduino on {port} at {baud} baud")
            
            # Send initial configuration
            self.configure_stepper()
            
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Failed to connect to Arduino: {str(e)}")
            self.serial_connection = None
            return
            
        # Create a thread for reading serial responses
        self.serial_thread = threading.Thread(target=self.read_serial_data)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        self.get_logger().info("✅ Simple Stepper Node Ready! Press:")
        self.get_logger().info("  🟢 A button - Start motor")
        self.get_logger().info("  🔴 B button - Stop motor")
        self.get_logger().info("  🔵 X button - Test motor (one rotation)")
        
    def configure_stepper(self):
        """Send initial stepper configuration to Arduino"""
        if not self.serial_connection:
            return
        try:
            # Configure for continuous operation: 200 steps, 800 speed, 400 accel, direction 1, continuous mode
            cmd = "C:200:800:400:1:1"
            self.send_command(cmd)
            time.sleep(0.1)  # Give Arduino time to process
            self.get_logger().info("✅ Stepper configured for continuous operation")
                
        except Exception as e:
            self.get_logger().error(f"❌ Failed to configure stepper: {str(e)}")
            
    def read_serial_data(self):
        """Continuously reads from serial port in a separate thread."""
        while rclpy.ok() and self.serial_connection:
            try:
                if self.serial_connection.in_waiting > 0:
                    line = self.serial_connection.readline().decode().strip()
                    if line:  # Only publish non-empty lines
                        self.get_logger().info(f"Arduino: {line}")
                        self.status_pub.publish(String(data=line))
            except Exception as e:
                self.get_logger().error(f"Error in serial read thread: {str(e)}")
                break
                
    def joy_callback(self, msg: Joy):
        """Handle joystick input for stepper control."""
        button_start_idx = self.get_parameter('button_start').value
        button_stop_idx = self.get_parameter('button_stop').value
        button_test_idx = self.get_parameter('button_test').value

        if len(msg.buttons) <= max(button_start_idx, button_stop_idx, button_test_idx):
            return

        start_pressed = msg.buttons[button_start_idx] == 1
        stop_pressed = msg.buttons[button_stop_idx] == 1
        test_pressed = msg.buttons[button_test_idx] == 1

        # Start motor (A button)
        if start_pressed and not self.prev_start_state and not self.stepper_running:
            self.get_logger().info('🟢 A BUTTON: Starting stepper motor continuously')
            self.send_command('R')
            self.stepper_running = True
        
        # Stop motor (B button)
        if stop_pressed and not self.prev_stop_state and self.stepper_running:
            self.get_logger().info('🔴 B BUTTON: Stopping stepper motor')
            self.send_command('S')
            self.stepper_running = False

        # Test motor (X button) - one rotation
        if test_pressed and not self.prev_test_state and not self.stepper_running:
            self.get_logger().info('🔵 X BUTTON: Testing motor (one rotation)')
            self.send_command('T')

        self.prev_start_state = start_pressed
        self.prev_stop_state = stop_pressed
        self.prev_test_state = test_pressed

    def send_command(self, command: str):
        """Send command to Arduino"""
        if self.serial_connection:
            with self.serial_lock:
                try:
                    self.serial_connection.write(f'{command}\n'.encode())
                    self.get_logger().info(f"📤 Sent command: {command}")
                except Exception as e:
                    self.get_logger().error(f"Failed to send command '{command}': {e}")
                    
    def request_status(self):
        """Request current status from Arduino"""
        self.send_command('P')
            
    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down stepper controller...")
        if self.serial_connection:
            try:
                self.send_command('S')  # Stop motor
                time.sleep(0.5)
                self.serial_connection.close()
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = StepperSimpleNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception) as e:
        print(f"❌ Node shutdown: {str(e)}")
    finally:
        if 'node' in locals() and rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main() 