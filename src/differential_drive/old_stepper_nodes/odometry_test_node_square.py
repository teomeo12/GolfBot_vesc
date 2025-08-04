#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import math
import time
import threading

class OdometryTestNodeEnhanced(Node):
    def __init__(self):
        super().__init__('odometry_test_node_enhanced')

        # --- Parameters ---
        # Using the same reliable ERPM values from the path follower
        self.drive_speed = 500.0
        self.turn_speed = 550.0
        self.is_running_sequence = False

        # --- Publishers and Subscribers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.current_pose = None
        self.lock = threading.Lock()

        self.get_logger().info("--- Enhanced Odometry Test Node Ready ---")
        self.get_logger().info('--> Press "A" button to start the complete square sequence (0.5m x 0.5m).')

    def odom_callback(self, msg: Odometry):
        with self.lock:
            self.current_pose = msg.pose.pose

    def joy_callback(self, msg: Joy):
        # Button 0 is 'A', Button 1 is 'B' on most gamepads
        a_button_pressed = msg.buttons[0] == 1
        b_button_pressed = msg.buttons[1] == 1
        
        if a_button_pressed and not self.is_running_sequence:
            self.get_logger().info("'A' button pressed. Starting simple drive sequence.")
            # Run the sequence in a separate thread to avoid blocking the ROS node
            sequence_thread = threading.Thread(target=self.run_sequence)
            sequence_thread.start()
            
        if b_button_pressed and self.is_running_sequence:
            self.get_logger().warn("--- 'B' BUTTON PRESSED: EMERGENCY STOP REQUESTED ---")
            self.is_running_sequence = False # This will cause the sequence loops to exit
            self.stop_robot()


    def run_sequence(self):
        self.is_running_sequence = True

        if self.current_pose is None:
            self.get_logger().error("Cannot start sequence, no odometry data received yet.")
            self.is_running_sequence = False
            return

        # Step 1: Drive forward 50cm
        self.get_logger().info("Step 1: Driving forward 0.5m...")
        self.drive_distance(0.5)
        
        # Step 2: Turn right 90 degrees
        self.stop_robot()
        self.get_logger().info("Step 2: Turning right 90 degrees...")
        time.sleep(0.5)
        self.turn_angle(-90.0) # Negative for right turn

        # Step 3: Drive forward 50cm
        self.stop_robot()
        self.get_logger().info("Step 3: Driving forward 0.5m...")
        time.sleep(0.5)
        self.drive_distance(0.5)

        # Step 4: Turn right 90 degrees
        self.stop_robot()
        self.get_logger().info("Step 4: Turning right 90 degrees...")
        time.sleep(0.5)
        self.turn_angle(-90.0)

        # Step 5: Drive forward 50cm
        self.stop_robot()
        self.get_logger().info("Step 5: Driving forward 0.5m...")
        time.sleep(0.5)
        self.drive_distance(0.5)

        # Step 6: Turn right 90 degrees (back to start orientation)
        self.stop_robot()
        self.get_logger().info("Step 6: Final turn right 90 degrees...")
        time.sleep(0.5)
        self.turn_angle(-90.0)

        # Step 7: Drive forward 50cm (back to start position)
        self.stop_robot()
        self.get_logger().info("Step 7: Driving to start position...")
        time.sleep(0.5)
        self.drive_distance(0.5)

        # Step 8: Final turn to original orientation
        self.stop_robot()
        self.get_logger().info("Step 8: Final turn to original orientation...")
        time.sleep(0.5)
        self.turn_angle(-90.0)

        # Final stop - ensure robot is completely stopped
        self.get_logger().info("--- SQUARE COMPLETE! Back at start position ---")
        self.stop_robot()
        time.sleep(0.5)  # Additional stop time
        
        # Send multiple stop commands to ensure robot stops completely
        for _ in range(5):
            self.stop_robot()
            time.sleep(0.1)
            
        self.get_logger().info("Robot fully stopped. Press A button to run the square sequence again.")
        self.is_running_sequence = False

    def drive_distance(self, distance):
        with self.lock:
            start_pose = self.current_pose
        
        distance_traveled = 0.0
        twist_msg = Twist()
        twist_msg.linear.x = self.drive_speed
        
        while distance_traveled < distance and self.is_running_sequence:
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.02) # Loop at 20Hz
            with self.lock:
                current_pose = self.current_pose
            distance_traveled = math.sqrt(
                (current_pose.position.x - start_pose.position.x)**2 +
                (current_pose.position.y - start_pose.position.y)**2
            )

    def turn_angle(self, angle_degrees):
        target_rad = math.radians(angle_degrees)
        
        with self.lock:
            start_yaw = self.quat_to_yaw(self.current_pose.orientation)

        turn_rad_traveled = 0.0
        twist_msg = Twist()
        twist_msg.angular.z = math.copysign(self.turn_speed, angle_degrees)
        
        while abs(turn_rad_traveled) < abs(target_rad) and self.is_running_sequence:
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.05) # Loop at 20Hz
            with self.lock:
                current_yaw = self.quat_to_yaw(self.current_pose.orientation)
            turn_rad_traveled = self.normalize_angle(current_yaw - start_yaw)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())
    
    def quat_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = OdometryTestNodeEnhanced()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 