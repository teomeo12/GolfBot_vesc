#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import threading
import time
from typing import Optional

class StepperControllerNode(Node):
    def __init__(self):
        super().__init__('stepper_controller_node')

        # Declare parameters with defaults
        self.declare_parameter('serial_port', '/dev/tty_arduino')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('button_start', 0)  # A button on Logitech F710 - start motor
        self.declare_parameter('button_stop', 1)   # B button on Logitech F710 - stop motor
        self.declare_parameter('steps_per_command', 800)
        self.declare_parameter('continuous_steps', 999999)  # Large number for continuous running
        self.declare_parameter('max_speed', 4000.0)
        self.declare_parameter('acceleration', 2000.0)
        self.declare_parameter('direction', 1)  # 1 for forward, -1 for reverse
        self.declare_parameter('auto_configure', True)  # Send config on startup
        self.declare_parameter('connection_timeout', 5.0)  # seconds
        self.declare_parameter('command_timeout', 2.0)  # seconds

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.button_start = self.get_parameter('button_start').get_parameter_value().integer_value
        self.button_stop = self.get_parameter('button_stop').get_parameter_value().integer_value
        self.steps_per_command = self.get_parameter('steps_per_command').get_parameter_value().integer_value
        self.continuous_steps = self.get_parameter('continuous_steps').get_parameter_value().integer_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.acceleration = self.get_parameter('acceleration').get_parameter_value().double_value
        self.direction = self.get_parameter('direction').get_parameter_value().integer_value
        self.auto_configure = self.get_parameter('auto_configure').get_parameter_value().bool_value
        self.connection_timeout = self.get_parameter('connection_timeout').get_parameter_value().double_value
        self.command_timeout = self.get_parameter('command_timeout').get_parameter_value().double_value

        # Serial connection
        self.serial_connection: Optional[serial.Serial] = None
        self.serial_lock = threading.Lock()
        self.connection_active = False

        # Button state tracking for debouncing
        self.prev_start_state = False
        self.prev_stop_state = False
        self.last_command_time = 0.0
        self.min_command_interval = 0.5  # Minimum 0.5 second between commands
        self.stepper_running = False  # Track if stepper is currently running

        # Create subscriber for Joy messages
        self.joy_subscriber = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        # Initialize serial connection
        self.connect_to_arduino()

        # Configure Arduino if requested
        if self.auto_configure and self.connection_active:
            self.configure_arduino()

        self.get_logger().info('Stepper Controller Node Started')
        self.get_logger().info(f'Serial port: {self.serial_port}')
        self.get_logger().info(f'Start button: {self.button_start} (A button)')
        self.get_logger().info(f'Stop button: {self.button_stop} (B button)')
        self.get_logger().info(f'Steps per command: {self.steps_per_command}')
        self.get_logger().info(f'Max speed: {self.max_speed}')
        self.get_logger().info(f'Acceleration: {self.acceleration}')
        self.get_logger().info(f'Direction: {"Forward" if self.direction == 1 else "Reverse"}')

    def connect_to_arduino(self):
        """Establish serial connection to Arduino"""
        try:
            self.get_logger().info(f'Attempting to connect to Arduino at {self.serial_port}...')
            
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.command_timeout,
                write_timeout=self.command_timeout
            )
            
            # Serial port opened successfully, mark as connected
            self.connection_active = True
            
            # Wait for Arduino to complete startup sequence
            self.get_logger().info('Waiting for Arduino startup sequence to complete...')
            time.sleep(4.0)  # Longer wait for full startup
            
            # Clear all startup messages
            self.get_logger().info('Clearing Arduino startup messages...')
            self.serial_connection.reset_input_buffer()
            
            # Give Arduino a moment to be ready for commands
            time.sleep(1.0)
            
            # Test connection by requesting status
            self.get_logger().info('Testing Arduino communication...')
            if self.send_command('STATUS', expect_response=True):
                self.get_logger().info('Successfully connected to Arduino')
            else:
                self.get_logger().warn('Arduino not responding properly to commands')
                # Keep connection active anyway - maybe it will work for motor commands
                
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {str(e)}')
            self.connection_active = False
            self.serial_connection = None

    def configure_arduino(self):
        """Send configuration parameters to Arduino"""
        if not self.connection_active:
            self.get_logger().warn('Cannot configure Arduino - not connected')
            return False

        config_command = f'C:{self.steps_per_command}:{self.max_speed}:{self.acceleration}:{self.direction}'
        
        self.get_logger().info(f'Configuring Arduino: {config_command}')
        
        if self.send_command(config_command, expect_response=True):
            # Save configuration to EEPROM
            if self.send_command('SAVE', expect_response=True):
                self.get_logger().info('Arduino configured and settings saved')
                return True
            else:
                self.get_logger().warn('Configuration sent but failed to save to EEPROM')
                return False
        else:
            self.get_logger().error('Failed to configure Arduino')
            return False

    def send_command(self, command: str, expect_response: bool = False) -> bool:
        """Send a command to Arduino via serial"""
        if not self.connection_active or not self.serial_connection:
            self.get_logger().error('Cannot send command - Arduino not connected')
            return False

        try:
            with self.serial_lock:
                # Send command (same as test script)
                command_with_newline = command + '\n'
                self.serial_connection.write(command_with_newline.encode('utf-8'))
                self.serial_connection.flush()
                
                self.get_logger().debug(f'Sent command: {command}')
                
                if expect_response:
                    # Simple approach like test script - just read a few lines
                    response_lines = []
                    for i in range(10):  # Read up to 10 lines
                        try:
                            line = self.serial_connection.readline().decode('utf-8').strip()
                            if line:
                                response_lines.append(line)
                                self.get_logger().debug(f'Arduino response: {line}')
                                
                                # For STATUS command, look for the end marker
                                if command == 'STATUS' and '==============' in line:
                                    break
                                # For other commands, look for completion
                                elif any(keyword in line for keyword in ['DONE', 'CONFIG_UPDATED', 'SAVED']):
                                    if 'DONE' in line:
                                        self.stepper_running = False
                                    break
                                elif 'ERROR' in line:
                                    self.get_logger().error(f'Arduino error: {line}')
                                    return False
                        except Exception as e:
                            self.get_logger().debug(f'Error reading line: {e}')
                            break
                    
                    if response_lines:
                        return True
                    else:
                        self.get_logger().warn(f'No response from Arduino for command: {command}')
                        return False
                else:
                    return True
                    
        except Exception as e:
            self.get_logger().error(f'Serial communication error: {str(e)}')
            self.connection_active = False
            return False

    def joy_callback(self, msg: Joy):
        """Handle joystick input"""
        # Check if we have enough buttons
        max_button = max(self.button_start, self.button_stop)
        if len(msg.buttons) <= max_button:
            self.get_logger().warn(f'Not enough buttons in Joy message. Expected at least {max_button + 1}')
            return

        # Get button states
        start_pressed = msg.buttons[self.button_start] == 1
        stop_pressed = msg.buttons[self.button_stop] == 1

        current_time = time.time()
        
        # Handle A button (Start motor)
        if (start_pressed and not self.prev_start_state and 
            current_time - self.last_command_time > self.min_command_interval):
            
            if not self.stepper_running:
                self.get_logger().info('🟢 A BUTTON: Starting stepper motor continuously')
                if self.start_continuous():
                    self.stepper_running = True
                    self.last_command_time = current_time
                else:
                    self.get_logger().error('Failed to start stepper motor')
            else:
                self.get_logger().info('A button pressed but motor already running')

        # Handle B button (Stop motor)
        if (stop_pressed and not self.prev_stop_state and 
            current_time - self.last_command_time > self.min_command_interval):
            
            if self.stepper_running:
                self.get_logger().info('🔴 B BUTTON: Stopping stepper motor')
                if self.stop_stepper():
                    self.stepper_running = False
                    self.last_command_time = current_time
                else:
                    self.get_logger().error('Failed to stop stepper motor')
            else:
                self.get_logger().info('B button pressed but motor already stopped')

        # Update previous states
        self.prev_start_state = start_pressed
        self.prev_stop_state = stop_pressed

    def trigger_stepper(self) -> bool:
        """Send run command to stepper motor"""
        if not self.connection_active:
            self.get_logger().error('Cannot trigger stepper - Arduino not connected')
            return False

        return self.send_command('R', expect_response=True)

    def start_continuous(self) -> bool:
        """Send continuous run command to stepper motor"""
        if not self.connection_active:
            self.get_logger().error('Cannot start continuous - Arduino not connected')
            return False

        return self.send_command('RCON', expect_response=True)

    def stop_stepper(self) -> bool:
        """Send stop command to stepper motor"""
        if not self.connection_active:
            self.get_logger().error('Cannot stop stepper - Arduino not connected')
            return False

        return self.send_command('S', expect_response=True)

    def get_arduino_status(self) -> bool:
        """Request status from Arduino"""
        if not self.connection_active:
            self.get_logger().error('Cannot get status - Arduino not connected')
            return False

        return self.send_command('STATUS', expect_response=True)

    def reconfigure_arduino(self, steps: Optional[int] = None, speed: Optional[float] = None, 
                          accel: Optional[float] = None, direction: Optional[int] = None) -> bool:
        """Reconfigure Arduino parameters at runtime"""
        if not self.connection_active:
            self.get_logger().error('Cannot reconfigure - Arduino not connected')
            return False

        # Use current values if not specified
        new_steps = steps if steps is not None else self.steps_per_command
        new_speed = speed if speed is not None else self.max_speed
        new_accel = accel if accel is not None else self.acceleration
        new_dir = direction if direction is not None else self.direction

        config_command = f'C:{new_steps}:{new_speed}:{new_accel}:{new_dir}'
        
        self.get_logger().info(f'Reconfiguring Arduino: {config_command}')
        
        if self.send_command(config_command, expect_response=True):
            # Update stored parameters
            self.steps_per_command = new_steps
            self.max_speed = new_speed
            self.acceleration = new_accel
            self.direction = new_dir
            return True
        else:
            return False

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down Stepper Controller Node')
        
        if self.connection_active and self.serial_connection:
            try:
                # Stop any running stepper
                self.send_command('S')
                time.sleep(0.1)
                self.serial_connection.close()
            except:
                pass
                
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = StepperControllerNode()
    
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