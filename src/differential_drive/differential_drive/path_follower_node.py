#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math
import time
from enum import Enum

class PathState(Enum):
    IDLE = 1
    TURNING_TO_WAYPOINT = 2
    DRIVING_TO_WAYPOINT = 3
    PATH_COMPLETE = 4

class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower_node')
        
        # --- State Management ---
        self.state = PathState.IDLE
        self.current_pose = None
        self.waypoints = []
        self.current_waypoint_index = 0
        
        # --- Parameters ---
        # Using D-pad (CORRECTED for flipped axes): 
        # Axis 6: +1 = LEFT (mower), -1 = RIGHT (square) 
        # Axis 7: +1 = UP (cancel), -1 = DOWN (unused for now)
        self.dpad_horizontal = 6  # Left/Right axis
        self.dpad_vertical = 7    # Up/Down axis
        self.turn_kp = 1.0            # Proportional gain for turning
        self.drive_speed = 35.0       # Similar to align_and_repair speeds
        self.turn_speed_max = 50.0    # Similar to align_and_repair speeds
        self.arrival_threshold = 0.1  # 10cm
        self.angle_threshold = 0.1    # ~6 degrees in radians
        
        # --- ROS2 Publishers and Subscribers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.control_timer = self.create_timer(0.05, self.control_loop) # 20 Hz
        
        self.get_logger().info("Path Follower Node Ready.")
        self.get_logger().info("D-pad: LEFT=Start Mower, RIGHT=Stop Mower, UP=Start Square, DOWN=Stop Square")
        self.prev_dpad_horizontal = 0.0
        self.prev_dpad_vertical = 0.0

    def joy_callback(self, msg: Joy):
        # Make sure we have enough axes
        if len(msg.axes) <= max(self.dpad_horizontal, self.dpad_vertical):
            return
            
        current_horizontal = msg.axes[self.dpad_horizontal]  # Axis 6
        current_vertical = msg.axes[self.dpad_vertical]      # Axis 7
        
        # --- HORIZONTAL CONTROLS (Mower) - CORRECTED AXES ---
        # D-pad LEFT pressed (start mower) - NOW +1 due to flipped axis
        if current_horizontal > 0.5 and abs(self.prev_dpad_horizontal) < 0.5:
            if self.state == PathState.IDLE:
                self.get_logger().info("🟩 D-pad LEFT pressed. Starting mower search pattern.")
                self.generate_mower_path()
            else:
                self.get_logger().info("D-pad LEFT pressed, but already running a path.")
        
        # D-pad RIGHT pressed (stop mower) - NOW -1 due to flipped axis
        elif current_horizontal < -0.5 and abs(self.prev_dpad_horizontal) < 0.5:
            if self.state != PathState.IDLE:
                self.get_logger().info("🛑 D-pad RIGHT pressed. STOPPING mower search pattern.")
                self.stop_robot()
                self.state = PathState.IDLE
                self.waypoints = []
                self.current_waypoint_index = 0
            else:
                self.get_logger().info("D-pad RIGHT pressed, but already in IDLE mode.")
        
        # --- VERTICAL CONTROLS (Square/Waypoint) - CORRECTED AXES ---
        # D-pad UP pressed (start square waypoint) - NOW +1 due to flipped axis
        if current_vertical > 0.5 and abs(self.prev_dpad_vertical) < 0.5:
            if self.state == PathState.IDLE:
                self.get_logger().info("🟦 D-pad UP pressed. Starting square waypoint pattern.")
                self.generate_square_path()
            else:
                self.get_logger().info("D-pad UP pressed, but already running a path.")
        
        # D-pad DOWN pressed (stop waypoint) - NOW -1 due to flipped axis
        elif current_vertical < -0.5 and abs(self.prev_dpad_vertical) < 0.5:
            if self.state != PathState.IDLE:
                self.get_logger().info("🛑 D-pad DOWN pressed. STOPPING waypoint pattern.")
                self.stop_robot()
                self.state = PathState.IDLE
                self.waypoints = []
                self.current_waypoint_index = 0
            else:
                self.get_logger().info("D-pad DOWN pressed, but already in IDLE mode.")
        
        # Store previous states
        self.prev_dpad_horizontal = current_horizontal
        self.prev_dpad_vertical = current_vertical

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def generate_square_path(self):
        if self.current_pose is None:
            self.get_logger().warn("Cannot start path, odometry not received yet.")
            return
            
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        
        self.waypoints = [
            (start_x + 1.0, start_y),
            (start_x + 1.0, start_y + 1.0),
            (start_x, start_y + 1.0),
            (start_x, start_y)
        ]
        self.start_path()

    def generate_mower_path(self):
        if self.current_pose is None:
            self.get_logger().warn("Cannot start path, odometry not received yet.")
            return

        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        
        self.waypoints = [
            (start_x + 2.0, start_y),
            (start_x + 2.0, start_y + 0.5),
            (start_x, start_y + 0.5),
            (start_x, start_y + 1.0),
            (start_x + 2.0, start_y + 1.0)
        ]
        self.start_path()

    def start_path(self):
        self.current_waypoint_index = 0
        self.state = PathState.TURNING_TO_WAYPOINT
        self.get_logger().info(f"Path started. First waypoint: {self.waypoints[0]}")

    def control_loop(self):
        if self.state == PathState.IDLE or self.current_pose is None:
            return

        if self.state == PathState.PATH_COMPLETE:
            self.get_logger().info("Path complete. Returning to IDLE.")
            self.state = PathState.IDLE
            return

        target_x, target_y = self.waypoints[self.current_waypoint_index]
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # --- Calculate angle to target ---
        angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
        current_yaw = self.quat_to_yaw(self.current_pose.orientation)
        angle_error = self.normalize_angle(angle_to_target - current_yaw)
        
        # --- State Logic ---
        if self.state == PathState.TURNING_TO_WAYPOINT:
            if abs(angle_error) < self.angle_threshold:
                self.stop_robot()
                self.state = PathState.DRIVING_TO_WAYPOINT
            else:
                self.turn_robot(angle_error)
        
        elif self.state == PathState.DRIVING_TO_WAYPOINT:
            distance_to_target = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
            
            # If we are facing away from the target, we must re-turn
            if abs(angle_error) > math.pi / 2:
                self.stop_robot()
                self.state = PathState.TURNING_TO_WAYPOINT
                return

            if distance_to_target < self.arrival_threshold:
                self.stop_robot()
                self.current_waypoint_index += 1
                if self.current_waypoint_index >= len(self.waypoints):
                    self.state = PathState.PATH_COMPLETE
                else:
                    self.get_logger().info(f"Waypoint {self.current_waypoint_index-1} reached. Turning to next waypoint.")
                    self.state = PathState.TURNING_TO_WAYPOINT
            else:
                self.drive_robot()

    def turn_robot(self, angle_error):
        twist = Twist()
        twist.linear.x = 0.0
        turn_speed = self.turn_kp * angle_error
        twist.angular.z = max(-self.turn_speed_max, min(self.turn_speed_max, turn_speed))
        self.cmd_vel_pub.publish(twist)

    def drive_robot(self):
        twist = Twist()
        twist.linear.x = self.drive_speed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

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
    path_follower_node = PathFollowerNode()
    rclpy.spin(path_follower_node)
    path_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 