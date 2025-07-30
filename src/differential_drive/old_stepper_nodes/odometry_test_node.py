#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import math

class OdometryTestNode(Node):
    def __init__(self):
        super().__init__('odometry_test_node')

        # --- Parameters ---
        self.target_distance = 1.0  # meters
        # NOTE: This speed is a raw ERPM value sent to the VESC, not m/s.
        # This matches the method used in the align_and_repair_node.
        self.move_speed = 55.0  # ERPM

        # --- State Variables ---
        self.state = 'IDLE'  # IDLE, MOVING, DONE
        self.start_x = 0.0
        self.current_x = 0.0
        self.initial_odom_received = False

        # --- Publishers and Subscribers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # A timer to control the movement logic
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Odometry Test Node Ready.')
        self.get_logger().info('Press the "A" button on your gamepad to start the 1-meter test.')
        self.get_logger().info('Ensure odometry is publishing and the robot is ready.')


    def odom_callback(self, msg: Odometry):
        """Updates the robot's current x position from odometry."""
        if not self.initial_odom_received:
            self.start_x = msg.pose.pose.position.x
            self.initial_odom_received = True
        self.current_x = msg.pose.pose.position.x

    def joy_callback(self, msg: Joy):
        """Listens for the 'A' button to start the test."""
        # Note: Button mapping can vary. buttons[0] is 'A', buttons[1] is 'B' on Xbox/Logitech.
        a_button_pressed = msg.buttons[0] == 1
        b_button_pressed = msg.buttons[1] == 1

        if a_button_pressed and self.state == 'IDLE':
            self.get_logger().info('A button pressed. Starting 1-meter forward test...')
            self.state = 'MOVING'
            # Record the starting position at the moment the test begins
            self.start_x = self.current_x
        
        if b_button_pressed and self.state == 'MOVING':
            self.get_logger().warn('B button pressed. Emergency stop requested.')
            self.stop_robot()
            self.state = 'IDLE' # Reset to allow a new test run


    def control_loop(self):
        """Main logic loop for controlling the robot's movement."""
        if self.state == 'MOVING':
            distance_traveled = abs(self.current_x - self.start_x)

            if distance_traveled < self.target_distance:
                # Still moving towards the target
                twist_msg = Twist()
                twist_msg.linear.x = self.move_speed
                self.cmd_vel_pub.publish(twist_msg)
            else:
                # Target distance reached
                self.stop_robot()
                self.get_logger().info(f'Test complete. Total distance traveled: {distance_traveled:.3f} meters.')
                self.state = 'DONE'

    def stop_robot(self):
        """Publishes a zero-velocity Twist message to stop the robot."""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info('Robot stopped.')


def main(args=None):
    rclpy.init(args=args)
    odometry_test_node = OdometryTestNode()
    try:
        rclpy.spin(odometry_test_node)
    except KeyboardInterrupt:
        pass
    finally:
        odometry_test_node.stop_robot()
        odometry_test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 