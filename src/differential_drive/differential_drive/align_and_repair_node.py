#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time
from enum import Enum

class AlignmentState(Enum):
    IDLE = 1
    ALIGNING = 2 # A single, unified state for both turning and distance adjustment
    PAUSING = 3
    WAITING = 4
    DRIVING = 5
    DISPENSING = 6

class AlignAndRepairNode(Node):
    def __init__(self):
        super().__init__('align_and_repair_node')
        
        # State management
        self.state = AlignmentState.IDLE
        self.autonomous_mode = False
        self.target_locked = False
        self.locked_target_position = None
        self.wait_start_time = None
        self.pause_start_time = None
        self.default_pause_duration = 5.0  # default pause duration (seconds)
        self.next_state_after_pause = None
        
        # --- Odometry Data ---
        self.current_pose = None
        self.target_destination = None # The (x, y) coordinate we want to drive to
        
        # Camera parameters (RealSense D435i defaults)
        self.camera_fx = 615.0  # Focal length X
        self.camera_fy = 615.0  # Focal length Y
        self.camera_cx = 320.0  # Principal point X
        self.camera_cy = 240.0  # Principal point Y
        self.center_threshold = 10  # Pixels from center to consider aligned
        
        # Physical measurements (in meters)
        self.dispenser_offset = 0.90  # Dispenser is 90cm behind camera
        
        # --- Proportional Control Parameters for smooth alignment ---
        # NOTE: The key to preventing oscillation is a low minimum speed.
        # The robot should be able to make very small, gentle adjustments.
        
        # Turning Control (Angular)
        self.turn_kp = 0.09             # A gentle gain to prevent aggressive reactions.
        self.min_turn_speed = 35.0      # A low minimum speed for fine-grained adjustments.
        self.max_turn_speed = 55.0      # Maximum speed to prevent runaway turning.

        # Alignment Driving Control (Linear)
        self.drive_kp = 0.09             # Proportional gain for forward/backward alignment.
        self.min_drive_speed = 25.0     # A low minimum speed for fine-grained adjustments.
        self.max_drive_speed = 30.0     # Maximum speed for fine alignment.

        # --- Final Drive Parameters ---
        self.final_drive_speed = 38.0   # A fixed speed for the final long drive.
        
        # Detection data
        self.latest_detection_info = None
        self.latest_depth_image = None
        self.bridge = CvBridge()
        
        # Subscribers
        self.autonomous_mode_sub = self.create_subscription(
            Bool,
            '/is_autonomous_mode',
            self.autonomous_mode_callback,
            10)
        
        self.detection_sub = self.create_subscription(
            String,
            '/camera/divot_detection/details',
            self.detection_callback,
            10)
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10)
            
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dispense_pub = self.create_publisher(String, '/dispense_sand', 10)
        self.autonomous_mode_pub = self.create_publisher(Bool, '/is_autonomous_mode', 10)
        
        # Control timer (50 Hz for smoother control)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info('Align and Repair Node Ready')
        
    def autonomous_mode_callback(self, msg):
        """Handle autonomous mode activation/deactivation"""
        if msg.data and not self.autonomous_mode:
            self.autonomous_mode = True
            self.get_logger().info('ENTERING AUTONOMOUS MODE: Starting sequence.')
            self.state = AlignmentState.ALIGNING
        elif not msg.data and self.autonomous_mode:
            self.autonomous_mode = False
            self.get_logger().info('EXITING AUTONOMOUS MODE: Robot stopped.')
            self.stop_robot()
            self.reset_sequence()
            
    def detection_callback(self, msg):
        """Parse detection data from YOLO"""
        if not msg.data:
            self.latest_detection_info = None
            return
            
        all_detections = msg.data.split(';')
        for detection_str in all_detections:
            parts = detection_str.split(',')
            if len(parts) < 4: 
                continue

            detection_info = {}
            for part in parts:
                key, value = part.split(':', 1)
                detection_info[key.strip()] = value.strip()
            
            if detection_info.get('class') == 'divot' and float(detection_info.get('confidence', 0)) > 0.5:
                detection_info['confidence'] = float(detection_info['confidence'])
                detection_info['center_x'] = int(detection_info['center_x'])
                detection_info['center_y'] = int(detection_info['center_y'])
                
                self.latest_detection_info = detection_info
                break
        
    def depth_callback(self, msg):
        """Store latest depth image"""
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

    def odom_callback(self, msg):
        """Store the latest odometry data"""
        self.current_pose = msg.pose.pose
            
    def get_divot_world_position(self, pixel_x, pixel_y):
        """Convert pixel coordinates to world coordinates relative to robot base"""
        if self.latest_depth_image is None:
            return None
            
        # Get depth at pixel location (in mm, convert to meters)
        depth = self.latest_depth_image[int(pixel_y), int(pixel_x)] / 1000.0
        
        if depth == 0 or depth > 5.0:  # Invalid depth reading
            return None
            
        # Convert pixel to camera coordinates
        cam_x = (pixel_x - self.camera_cx) * depth / self.camera_fx
        cam_y = (pixel_y - self.camera_cy) * depth / self.camera_fy
        cam_z = depth
        
        # Transform to robot base_link frame
        # In the robot's coordinate system (base_link):
        # - The X-axis points forward.
        # - The Y-axis points to the left.
        # - The Z-axis points up.
        # The camera's Z-axis (depth) corresponds to the robot's X-axis (forward).
        forward_dist = cam_z
        lateral_offset = -cam_x
        
        return {'forward_dist': forward_dist, 'lateral_offset': lateral_offset}
        
    def control_loop(self):
        """Main control loop called at 10 Hz"""
        if not self.autonomous_mode:
            return
            
        if self.state == AlignmentState.ALIGNING:
            self.handle_aligning()
        elif self.state == AlignmentState.PAUSING:
            self.handle_pausing()
        elif self.state == AlignmentState.WAITING:
            self.handle_waiting()
        elif self.state == AlignmentState.DRIVING:
            self.handle_driving()
        elif self.state == AlignmentState.DISPENSING:
            self.handle_dispensing()
            
    def handle_aligning(self):
        """A unified controller to handle both turning and distance adjustment simultaneously."""
        if not hasattr(self, 'aligning_logged'):
            self.get_logger().info('STATE: ALIGNING - Simultaneously turning and adjusting distance.')
            self.aligning_logged = True

        if self.latest_detection_info is None:
            self.stop_robot()
            return

        pixel_error_x = self.latest_detection_info['center_x'] - self.camera_cx
        pixel_error_y = self.latest_detection_info['center_y'] - self.camera_cy

        # Check if both errors are within the threshold
        if abs(pixel_error_x) < self.center_threshold and abs(pixel_error_y) < self.center_threshold:
            self.get_logger().info('Full alignment complete.')
            self.stop_robot()
            self.state = AlignmentState.WAITING
            self.wait_start_time = time.time()
            return

        # --- Calculate Angular (Turning) Speed ---
        turn_speed = self.turn_kp * pixel_error_x
        if 0 < abs(turn_speed) < self.min_turn_speed:
            turn_speed = self.min_turn_speed if turn_speed > 0 else -self.min_turn_speed
        turn_speed = max(-self.max_turn_speed, min(self.max_turn_speed, turn_speed))

        # --- Calculate Linear (Driving) Speed ---
        drive_speed = self.drive_kp * pixel_error_y
        if 0 < abs(drive_speed) < self.min_drive_speed:
            drive_speed = self.min_drive_speed if drive_speed > 0 else -self.min_drive_speed
        drive_speed = max(-self.max_drive_speed, min(self.max_drive_speed, drive_speed))

        # --- Create and Publish Combined Command ---
        twist = Twist()
        twist.angular.z = -turn_speed # To turn right (positive pixel_error_x), we need a negative angular.z
        twist.linear.x = -drive_speed # If divot is below center (positive pixel_error_y), move forward (-speed).
        self.cmd_vel_pub.publish(twist)

    def start_pause(self, next_state, pause_secs=None):
        """Initiates a pause for the requested duration."""
        duration = pause_secs if pause_secs is not None else self.default_pause_duration
        self.state = AlignmentState.PAUSING
        self.pause_start_time = time.time()
        self.pause_duration = duration
        self.next_state_after_pause = next_state
        self.get_logger().info(f"PAUSING for {duration} seconds...")

    def handle_pausing(self):
        """Waits for the pause duration to elapse."""
        if time.time() - self.pause_start_time >= self.pause_duration:
            self.get_logger().info("Pause complete.")
            self.state = self.next_state_after_pause
            # If we are transitioning to the main WAITING state, start its timer
            if self.state == AlignmentState.WAITING:
                self.wait_start_time = time.time()
        
    def handle_waiting(self):
        """Final wait before calculating the fixed 90cm drive."""
        elapsed = time.time() - self.wait_start_time
        
        if elapsed >= 5.0:
            self.get_logger().info('Wait complete. Calculating target destination...')
            if self.current_pose is not None:
                # --- Calculate absolute target coordinate ---
                current_x = self.current_pose.position.x
                current_y = self.current_pose.position.y
                current_theta = self.quat_to_yaw(self.current_pose.orientation)

                # The distance to drive is a fixed 90cm, straight ahead.
                drive_dist = self.dispenser_offset
                
                # Calculate the target coordinate in the odom frame
                target_x = current_x + drive_dist * math.cos(current_theta)
                target_y = current_y + drive_dist * math.sin(current_theta)
                self.target_destination = Point(x=target_x, y=target_y, z=0.0)
                
                self.target_locked = True
                self.get_logger().info(f"TARGET LOCKED: Driving a fixed {drive_dist:.2f}m to ({target_x:.2f}, {target_y:.2f})")
                self.state = AlignmentState.DRIVING
            else:
                self.get_logger().error('LOCK FAILED: No odometry data. Resetting.')
                self.reset_sequence()
        
    def handle_driving(self):
        """Drive to a specific coordinate using odometry."""
        if not self.target_locked or self.current_pose is None or self.target_destination is None:
            self.get_logger().error("DRIVE FAILED: State is not ready for driving. Resetting.")
            self.stop_robot()
            self.reset_sequence()
            return
            
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        target_x = self.target_destination.x
        target_y = self.target_destination.y
        
        distance_to_target = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        
        # Check if we have arrived
        if distance_to_target < 0.05: # 5cm arrival threshold
            self.get_logger().info(f"Drive complete. Arrived at target ({target_x:.2f}, {target_y:.2f}).")
            self.stop_robot()
            self.state = AlignmentState.DISPENSING
            self.dispense_start_time = time.time()
            return
        
        # If still driving, send command to move forward
        twist = Twist()
        twist.linear.x = self.final_drive_speed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def handle_dispensing(self):
        """Start dispensing at target position"""
        if not hasattr(self, 'dispensing_logged'):
            self.get_logger().info('STATE: DISPENSING - Releasing sand.')
            self.dispensing_logged = True

        elapsed = time.time() - self.dispense_start_time
        
        if elapsed < 0.5:
            if not hasattr(self, 'dispense_started'):
                dispense_msg = String()
                dispense_msg.data = 'R'
                self.dispense_pub.publish(dispense_msg)
                self.dispense_started = True
                
        elif elapsed < 3.0:
            pass
            
        elif elapsed < 3.5:
            if not hasattr(self, 'dispense_stopped'):
                dispense_msg = String()
                dispense_msg.data = 'S'
                self.dispense_pub.publish(dispense_msg)
                self.dispense_stopped = True
                
        else:
            self.get_logger().info('Dispense complete. Sequence finished.')
            autonomous_msg = Bool()
            autonomous_msg.data = False
            self.autonomous_mode_pub.publish(autonomous_msg)
            self.reset_sequence()
                
    def stop_robot(self):
        """Send zero velocity command to stop the robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def reset_sequence(self):
        """Reset state for next run"""
        self.state = AlignmentState.IDLE
        self.autonomous_mode = False
        self.target_locked = False
        self.locked_target_position = None
        self.wait_start_time = None
        self.pause_start_time = None
        self.next_state_after_pause = None
        self.target_destination = None
        self.current_pose = None

        # Reset logging flags
        if hasattr(self, 'aligning_logged'): delattr(self, 'aligning_logged')
        if hasattr(self, 'waiting_logged'): delattr(self, 'waiting_logged')
        if hasattr(self, 'driving_logged'): delattr(self, 'driving_logged')
        if hasattr(self, 'dispensing_logged'): delattr(self, 'dispensing_logged')
        
        if hasattr(self, 'dispense_started'):
            delattr(self, 'dispense_started')
        if hasattr(self, 'dispense_stopped'):
            delattr(self, 'dispense_stopped')

    def quat_to_yaw(self, q: Quaternion):
        """Convert a geometry_msgs/Quaternion to a yaw angle in radians."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    
    node = AlignAndRepairNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Align and Repair Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()