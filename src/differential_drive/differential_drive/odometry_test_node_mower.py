#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, Imu
import math
import time
import threading

class OdometryTestNodeMower(Node):
    def __init__(self):
        super().__init__('odometry_test_node_mower')

        # --- Parameters ---
        # Using the same reliable ERPM values from the path follower
        self.drive_speed = 450.0
        self.turn_speed = 400.0
        self.is_running_sequence = False
        
        # Heading correction parameters
        self.heading_correction_gain = 600.0  # Angular velocity gain for heading correction

        # Mower pattern parameters
        self.forward_distance = 0.5  # 1 meter forward passes
        self.wheelbase_width = 0.20  # 45cm wheelbase width

        # --- Publishers and Subscribers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.current_pose = None
        self.current_imu_orientation = None
        self.lock = threading.Lock()

        self.get_logger().info("--- Mower Pattern Odometry Test Node Ready ---")
        self.get_logger().info('--> Press "A" button to start the lawn mower pattern sequence.')
        self.get_logger().info(f'--> Pattern: 1m forward passes with {self.wheelbase_width}m lateral spacing')

    def odom_callback(self, msg: Odometry):
        with self.lock:
            self.current_pose = msg.pose.pose
    
    def imu_callback(self, msg: Imu):
        with self.lock:
            self.current_imu_orientation = msg.orientation

    def joy_callback(self, msg: Joy):
        # Button 0 is 'A', Button 1 is 'B' on most gamepads
        a_button_pressed = msg.buttons[0] == 1
        b_button_pressed = msg.buttons[1] == 1
        
        if a_button_pressed and not self.is_running_sequence:
            self.get_logger().info("'A' button pressed. Starting mower pattern sequence.")
            # Run the sequence in a separate thread to avoid blocking the ROS node
            sequence_thread = threading.Thread(target=self.run_mower_sequence)
            sequence_thread.start()
            
        if b_button_pressed and self.is_running_sequence:
            self.get_logger().warn("--- 'B' BUTTON PRESSED: EMERGENCY STOP REQUESTED ---")
            self.is_running_sequence = False # This will cause the sequence loops to exit
            self.stop_robot()

    def run_mower_sequence(self):
        self.is_running_sequence = True

        if self.current_pose is None:
            self.get_logger().error("Cannot start sequence, no odometry data received yet.")
            self.is_running_sequence = False
            return

        # MOWER PATTERN SEQUENCE
        
        # Pass 1: Drive forward 1m
        self.get_logger().info("Pass 1: Driving forward 1.0m...")
        self.drive_distance(self.forward_distance)
        
        # Turn right 90° to face lateral direction
        self.stop_robot()
        self.get_logger().info("Turning right 90° (toward next lane)...")
        time.sleep(0.5)
        self.turn_angle(-85.0)  # Right turn

        # Move wheelbase width (57cm) to next lane
        self.stop_robot()
        self.get_logger().info(f"Moving {self.wheelbase_width}m to next lane...")
        time.sleep(0.5)
        self.drive_distance(self.wheelbase_width)

        # Turn right 90° to face opposite direction for return pass
        self.stop_robot()
        self.get_logger().info("Turning right 90° (for return pass)...")
        time.sleep(0.5)
        self.turn_angle(-85.0)  # Right turn

        # Pass 2: Drive forward 1m (return direction)
        self.stop_robot()
        self.get_logger().info("Pass 2: Driving forward 1.0m (return direction)...")
        time.sleep(0.5)
        self.drive_distance(self.forward_distance)

        # Turn left 90° to face lateral direction (opposite side)
        self.stop_robot()
        self.get_logger().info("Turning left 90° (toward next lane)...")
        time.sleep(0.5)
        self.turn_angle(85.0)  # Left turn

        # Move wheelbase width (57cm) to next lane
        self.stop_robot()
        self.get_logger().info(f"Moving {self.wheelbase_width}m to next lane...")
        time.sleep(0.5)
        self.drive_distance(self.wheelbase_width)

        # Turn left 90° to face forward direction again
        self.stop_robot()
        self.get_logger().info("Turning left 90° (ready for next pass)...")
        time.sleep(0.5)
        self.turn_angle(85.0)  # Left turn

        # Pass 3: Drive forward 1m (original direction)
        self.stop_robot()
        self.get_logger().info("Pass 3: Driving forward 1.0m (original direction)...")
        time.sleep(0.5)
        self.drive_distance(self.forward_distance)

        # Final stop - ensure robot is completely stopped
        self.get_logger().info("--- MOWER PATTERN COMPLETE! ---")
        self.get_logger().info("Robot completed 3 parallel passes with proper lane spacing.")
        self.stop_robot()
        time.sleep(0.5)  # Additional stop time
        
        # Send multiple stop commands to ensure robot stops completely
        for _ in range(5):
            self.stop_robot()
            time.sleep(0.1)
            
        self.get_logger().info("Robot fully stopped. Press A button to run the mower pattern again.")
        self.is_running_sequence = False

    def drive_distance(self, distance):
        # Wait for IMU data to be available for heading correction
        while self.current_imu_orientation is None and self.is_running_sequence:
            self.get_logger().warn("Waiting for IMU data for straight line driving...")
            time.sleep(0.1)
            
        if not self.is_running_sequence:
            return
            
        with self.lock:
            start_pose = self.current_pose
            start_yaw = self.quat_to_yaw(self.current_imu_orientation)
        
        distance_traveled = 0.0
        
        while distance_traveled < distance and self.is_running_sequence:
            # Calculate heading error
            with self.lock:
                current_yaw = self.quat_to_yaw(self.current_imu_orientation)
            heading_error = self.normalize_angle(current_yaw - start_yaw)
            
            # Create corrected twist command
            twist_msg = Twist()
            twist_msg.linear.x = self.drive_speed
            # Apply proportional heading correction (negative feedback)
            twist_msg.angular.z = -self.heading_correction_gain * heading_error
            
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.02) # Loop at 50Hz
            
            with self.lock:
                current_pose = self.current_pose
            distance_traveled = math.sqrt(
                (current_pose.position.x - start_pose.position.x)**2 +
                (current_pose.position.y - start_pose.position.y)**2
            )

    def turn_angle(self, angle_degrees):
        target_rad = math.radians(angle_degrees)
        
        # Wait for IMU data to be available
        while self.current_imu_orientation is None and self.is_running_sequence:
            self.get_logger().warn("Waiting for IMU data...")
            time.sleep(0.1)
        
        if not self.is_running_sequence:
            return
        
        with self.lock:
            start_yaw = self.quat_to_yaw(self.current_imu_orientation)
            
        self.get_logger().info(f"IMU Turn: Starting yaw = {math.degrees(start_yaw):.1f}°, Target = {angle_degrees}°")

        turn_rad_traveled = 0.0
        twist_msg = Twist()
        twist_msg.angular.z = math.copysign(self.turn_speed, angle_degrees)
        
        while abs(turn_rad_traveled) < abs(target_rad) and self.is_running_sequence:
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.02) # Loop at 50Hz
            with self.lock:
                current_yaw = self.quat_to_yaw(self.current_imu_orientation)
            turn_rad_traveled = self.normalize_angle(current_yaw - start_yaw)
            
        with self.lock:
            final_yaw = self.quat_to_yaw(self.current_imu_orientation)
        self.get_logger().info(f"IMU Turn: Final yaw = {math.degrees(final_yaw):.1f}°, Turned = {math.degrees(turn_rad_traveled):.1f}°")

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
    node = OdometryTestNodeMower()
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