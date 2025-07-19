#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import String
import serial
import math
import time
import threading
from typing import Optional

class StepperImuNode(Node):
    def __init__(self):
        super().__init__('stepper_imu_node')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('max_speed', 800.0)
        self.declare_parameter('acceleration', 400.0)
        self.declare_parameter('steps', 200)
        self.declare_parameter('direction', 1)
        self.declare_parameter('continuous', True) # Defaulting to True for Joy control
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('button_start', 0)  # A button
        self.declare_parameter('button_stop', 1)   # B button
        
        # Create publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.status_pub = self.create_publisher(String, 'stepper/status', 10)
        
        # Create Joy subscriber
        self.joy_subscriber = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        # Button state tracking
        self.stepper_running = False
        self.prev_start_state = False
        self.prev_stop_state = False

        # Initialize serial connection
        self.serial_connection: Optional[serial.Serial] = None
        self.serial_lock = threading.Lock()
        try:
            port = self.get_parameter('port').value
            baud = self.get_parameter('baud_rate').value
            if port is None or baud is None:
                raise serial.SerialException("Port or baud rate not set")
            self.serial_connection = serial.Serial(port, int(baud), timeout=1.0)
            time.sleep(2)  # Wait for Arduino to reset
            self.get_logger().info(f"🔌 Connected to Arduino on {port}")
            
            # Configure stepper with initial parameters
            self.configure_stepper()
            
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Failed to connect to Arduino: {str(e)}")
            self.serial_connection = None
            return
            
        # Create a thread for continuously reading serial data
        self.serial_thread = threading.Thread(target=self.read_serial_data)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        self.get_logger().info("✅ Stepper + IMU Node Ready! Listening for Joy commands.")
        
    def configure_stepper(self):
        """Send stepper configuration to Arduino"""
        if not self.serial_connection:
            return
        try:
            speed = self.get_parameter('max_speed').value
            accel = self.get_parameter('acceleration').value
            steps = self.get_parameter('steps').value
            direction = self.get_parameter('direction').value
            continuous = 1 if self.get_parameter('continuous').value else 0
            
            cmd = f"C:{steps}:{speed}:{accel}:{direction}:{continuous}"
            self.send_command(cmd)
            # No need to wait for "OK" with continuous streaming
            self.get_logger().info("✅ Stepper configuration sent.")
                
        except Exception as e:
            self.get_logger().error(f"❌ Failed to configure stepper: {str(e)}")
            
    def read_serial_data(self):
        """Continuously reads from serial port in a separate thread."""
        while rclpy.ok() and self.serial_connection:
            try:
                if self.serial_connection.in_waiting > 0:
                    line = self.serial_connection.readline().decode().strip()
                    if line.startswith('I:'):
                        self.handle_imu_data(line)
                    elif line: # Don't publish empty lines
                        self.status_pub.publish(String(data=line))
            except Exception as e:
                self.get_logger().error(f"Error in serial read thread: {str(e)}")
                break
                
    def handle_imu_data(self, data):
        """Parse and publish IMU data"""
        try:
            parts = data.split(':')[1:]
            if len(parts) != 13:
                self.get_logger().warn(f"Invalid IMU data format, got {len(parts)} parts: {data}")
                return
                
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.get_parameter('imu_frame_id').value
            
            # Orientation (Euler to Quaternion)
            heading = float(parts[0]) * math.pi / 180.0
            roll = float(parts[1]) * math.pi / 180.0
            pitch = float(parts[2]) * math.pi / 180.0
            cy = math.cos(heading * 0.5); sy = math.sin(heading * 0.5)
            cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
            msg.orientation.w = cy * cp * cr + sy * sp * sr
            msg.orientation.x = cy * cp * sr - sy * sp * cr
            msg.orientation.y = sy * cp * sr + cy * sp * cr
            msg.orientation.z = sy * cp * cr - cy * sp * sr
            
            # Linear Acceleration & Angular Velocity
            msg.linear_acceleration.x = float(parts[3])
            msg.linear_acceleration.y = float(parts[4])
            msg.linear_acceleration.z = float(parts[5])
            msg.angular_velocity.x = float(parts[6])
            msg.angular_velocity.y = float(parts[7])
            msg.angular_velocity.z = float(parts[8])
            
            msg.orientation_covariance = [-1.0] * 9
            msg.angular_velocity_covariance = [-1.0] * 9
            msg.linear_acceleration_covariance = [-1.0] * 9
            
            self.imu_pub.publish(msg)
            
        except (ValueError, IndexError) as e:
            self.get_logger().error(f"Error parsing IMU data: {str(e)} on line: {data}")

    def joy_callback(self, msg: Joy):
        """Handle joystick input for stepper control."""
        button_start_idx = self.get_parameter('button_start').value
        button_stop_idx = self.get_parameter('button_stop').value

        # Linter fix: Ensure button indices are valid integers
        if not isinstance(button_start_idx, int) or not isinstance(button_stop_idx, int):
            self.get_logger().warn("Stepper button parameters are not set correctly.")
            return

        if len(msg.buttons) <= max(button_start_idx, button_stop_idx):
            return

        start_pressed = msg.buttons[button_start_idx] == 1
        stop_pressed = msg.buttons[button_stop_idx] == 1

        if start_pressed and not self.prev_start_state and not self.stepper_running:
            self.get_logger().info('🟢 A BUTTON: Starting stepper motor continuously')
            self.send_command('R')
            self.stepper_running = True
        
        if stop_pressed and not self.prev_stop_state and self.stepper_running:
            self.get_logger().info('🔴 B BUTTON: Stopping stepper motor')
            self.send_command('S')
            self.stepper_running = False

        self.prev_start_state = start_pressed
        self.prev_stop_state = stop_pressed

    def send_command(self, command: str):
        if self.serial_connection:
            with self.serial_lock:
                try:
                    self.serial_connection.write(f'{command}\n'.encode())
                    self.get_logger().info(f"Sent command: {command}")
                except Exception as e:
                    self.get_logger().error(f"Failed to send command '{command}': {e}")
            
    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down...")
        if self.serial_connection:
            try:
                self.send_command('S') # Stop motor
                self.serial_connection.close()
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = StepperImuNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception) as e:
        print(f"❌ Node shutdown requested: {str(e)}")
    finally:
        if 'node' in locals() and rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main() 