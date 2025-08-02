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
    FINAL_DRIVE = 7  # Drive 10cm past divot after dispensing

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
        self.drive_start_x = None  # Simple: just track X position like odometry_test_node
        self.drive_distance = 0.85  # Fixed 85cm drive distance (matches dispenser_offset)
        self.final_drive_distance = 0.10  # Additional 10cm after dispensing
        
        # Camera parameters (RealSense D435i defaults)
        self.camera_fx = 615.0  # Focal length X
        self.camera_fy = 615.0  # Focal length Y
        self.camera_cx = 320.0  # Principal point X
        self.camera_cy = 240.0  # Principal point Y
        self.alignment_threshold_cm = 3  # Real-world distance threshold for alignment (cm) - increased for testing
        
        # Physical measurements (in meters)
        self.dispenser_offset = 0.90  # Dispenser is 90cm behind camera
        
        # --- Two-Speed Alignment Control Parameters ---
        # Coarse positioning speeds (when offset > 5cm)
        self.coarse_turn_speed = 550.0   # Fast turning speed for large corrections
        self.coarse_drive_speed = 550.0  # Fast driving speed for large corrections
        
        # Fine positioning speeds (when 1.5cm < offset <= 5cm)  
        self.fine_turn_speed = 450.0     # Gentle turning speed for fine alignment
        self.fine_drive_speed = 450.0    # Gentle driving speed for fine alignment
        
        # Speed transition threshold
        self.speed_transition_threshold_cm = 5.0  # Switch from coarse to fine at 5cm

        # --- Driving State Parameters ---
        self.drive_speed_max = 750.0     # Fast speed for initial drive phase
        self.drive_speed_min = 700.0     # Slow speed when approaching target
        self.drive_slowdown_distance = 0.10  # Start slowing down 10cm before target (0.80m - 0.10m = 0.70m)
        
        # --- Final Drive Parameters ---
        self.final_drive_speed = 750.0   # Speed that definitely works in auto mode
        
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
        
        # Control timer (20 Hz to reduce command flooding)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
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
        elif self.state == AlignmentState.FINAL_DRIVE:
            self.handle_final_drive()
            
    def handle_aligning(self):
        """A unified controller using real-world distance thresholds instead of pixels."""
        if not hasattr(self, 'aligning_logged'):
            self.get_logger().info('STATE: ALIGNING - Using 1.5cm real-world distance threshold.')
            self.aligning_logged = True

        if self.latest_detection_info is None or self.latest_depth_image is None:
            self.stop_robot()
            return

        center_x = self.latest_detection_info['center_x']
        center_y = self.latest_detection_info['center_y']
        
        # Get depth at divot center
        depth_mm = self.latest_depth_image[center_y, center_x]
        if depth_mm == 0 or depth_mm > 5000:  # Invalid depth
            self.get_logger().warn(f'Invalid depth reading: {depth_mm}mm at pixel ({center_x}, {center_y})')
            self.stop_robot()
            return
            
        depth_m = depth_mm / 1000.0
        
        # Calculate real-world offsets in cm
        h_offset_m = ((center_x - self.camera_cx) * depth_m) / self.camera_fx
        v_offset_m = ((center_y - self.camera_cy) * depth_m) / self.camera_fy
        h_offset_cm = abs(h_offset_m * 100)
        v_offset_cm = abs(v_offset_m * 100)

        # DEBUG: Always log alignment status
        self.get_logger().info(f'ALIGNMENT CHECK: H={h_offset_cm:.1f}cm, V={v_offset_cm:.1f}cm, Depth={depth_m:.2f}m, Threshold={self.alignment_threshold_cm}cm')

        # Check if both offsets are within the real-world threshold
        if h_offset_cm < self.alignment_threshold_cm and v_offset_cm < self.alignment_threshold_cm:
            self.get_logger().info(f'*** ALIGNMENT COMPLETE! *** H-offset: {h_offset_cm:.1f}cm, V-offset: {v_offset_cm:.1f}cm')
            self.stop_robot()
            self.state = AlignmentState.WAITING
            self.wait_start_time = time.time()
            return

        # Calculate pixel errors for control (same as before)
        pixel_error_x = center_x - self.camera_cx
        pixel_error_y = center_y - self.camera_cy

        # --- Two-speed turning control ---
        if h_offset_cm < self.alignment_threshold_cm:
            turn_speed = 0.0  # Stop turning when within threshold
        elif h_offset_cm > self.speed_transition_threshold_cm:  # Coarse positioning 
            turn_speed = self.coarse_turn_speed if pixel_error_x > 0 else -self.coarse_turn_speed
        else:  # Fine positioning
            turn_speed = self.fine_turn_speed if pixel_error_x > 0 else -self.fine_turn_speed

        # --- Two-speed driving control ---  
        if v_offset_cm < self.alignment_threshold_cm:
            drive_speed = 0.0  # Stop driving when within threshold
        elif v_offset_cm > self.speed_transition_threshold_cm:  # Coarse positioning
            drive_speed = self.coarse_drive_speed if pixel_error_y > 0 else -self.coarse_drive_speed  
        else:  # Fine positioning
            drive_speed = self.fine_drive_speed if pixel_error_y > 0 else -self.fine_drive_speed

        # --- Create and Publish Combined Command ---
        twist = Twist()
        twist.angular.z = -turn_speed # To turn right (positive pixel_error_x), we need a negative angular.z
        twist.linear.x = -drive_speed # If divot is below center (positive pixel_error_y), move forward (-speed).
        self.cmd_vel_pub.publish(twist)
        
        # Log current alignment status
        self.get_logger().info(f'Aligning: H-offset={h_offset_cm:.1f}cm, V-offset={v_offset_cm:.1f}cm (target: <{self.alignment_threshold_cm}cm)', throttle_duration_sec=1.0)

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
        """Final wait before starting simple 90cm drive."""
        elapsed = time.time() - self.wait_start_time
        
        if elapsed >= 5.0:
            self.get_logger().info('Wait complete. Starting simple 90cm drive...')
            if self.current_pose is not None:
                # Simple approach: just record starting X position
                self.drive_start_x = self.current_pose.position.x
                self.target_locked = True
                self.get_logger().info(f"DRIVE STARTED: From X={self.drive_start_x:.3f}, target distance={self.drive_distance:.2f}m")
                self.state = AlignmentState.DRIVING
            else:
                self.get_logger().error('DRIVE FAILED: No odometry data. Resetting.')
                self.reset_sequence()
        
    def handle_driving(self):
        """Simple drive using distance tracking like odometry_test_node."""
        if not self.target_locked or self.current_pose is None or self.drive_start_x is None:
            self.get_logger().error("DRIVE FAILED: State is not ready for driving. Resetting.")
            self.stop_robot()
            self.reset_sequence()
            return
            
        current_x = self.current_pose.position.x
        distance_traveled = abs(current_x - self.drive_start_x)
        
        # Check if we've driven the target distance
        if distance_traveled >= self.drive_distance:
            self.get_logger().info(f"Drive complete. Traveled {distance_traveled:.3f}m (target: {self.drive_distance:.2f}m)")
            self.stop_robot()
            self.state = AlignmentState.DISPENSING
            self.dispense_start_time = time.time()
            return
        
        # Calculate remaining distance to target
        remaining_distance = self.drive_distance - distance_traveled
        
        # Choose speed based on remaining distance
        if remaining_distance > self.drive_slowdown_distance:
            # Use fast speed when far from target
            drive_speed = self.drive_speed_max
        else:
            # Use slow speed when approaching target (within 10cm)
            drive_speed = self.drive_speed_min
            
        # Continue driving forward
        twist = Twist()
        twist.linear.x = drive_speed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def handle_dispensing(self):
        """Start dispensing at target position for 10 seconds"""
        if not hasattr(self, 'dispensing_logged'):
            self.get_logger().info('STATE: DISPENSING - Releasing sand for 10 seconds.')
            self.dispensing_logged = True

        elapsed = time.time() - self.dispense_start_time
        
        if elapsed < 0.5:
            if not hasattr(self, 'dispense_started'):
                dispense_msg = String()
                dispense_msg.data = 'R'
                self.dispense_pub.publish(dispense_msg)
                self.dispense_started = True
                self.get_logger().info('Dispensing started.')
                
        elif elapsed < 5.0:  # Dispense for 10 seconds
            pass  # Keep dispensing
            
        elif elapsed < 5.5:  # Stop dispensing
            if not hasattr(self, 'dispense_stopped'):
                dispense_msg = String()
                dispense_msg.data = 'S'
                self.dispense_pub.publish(dispense_msg)
                self.dispense_stopped = True
                self.get_logger().info('Dispensing stopped. Starting final 10cm drive.')
                
        else:
            # Start final drive phase
            if self.current_pose is not None:
                self.drive_start_x = self.current_pose.position.x
                self.get_logger().info(f'FINAL DRIVE: Starting 10cm drive from X={self.drive_start_x:.3f}')
                self.state = AlignmentState.FINAL_DRIVE
            else:
                self.get_logger().error('FINAL DRIVE FAILED: No odometry data. Finishing sequence.')
                self.finish_sequence()
                
    def handle_final_drive(self):
        """Drive additional 10cm past the divot after dispensing."""
        if not hasattr(self, 'final_drive_logged'):
            self.get_logger().info('STATE: FINAL DRIVE - Driving 10cm past divot.')
            self.final_drive_logged = True
            
        if self.current_pose is None or self.drive_start_x is None:
            self.get_logger().error("FINAL DRIVE FAILED: No odometry data. Finishing sequence.")
            self.finish_sequence()
            return
            
        current_x = self.current_pose.position.x
        distance_traveled = abs(current_x - self.drive_start_x)
        
        # Check if we've driven the additional 10cm
        if distance_traveled >= self.final_drive_distance:
            self.get_logger().info(f"Final drive complete. Traveled {distance_traveled:.3f}m (target: {self.final_drive_distance:.2f}m)")
            self.stop_robot()
            self.finish_sequence()
            return
        
        # Continue driving forward
        twist = Twist()
        twist.linear.x = self.final_drive_speed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
    def finish_sequence(self):
        """Complete the sequence and return to manual mode."""
        self.get_logger().info('Full sequence complete! Returning to manual mode.')
        self.stop_robot()
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
        self.drive_start_x = None
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
        if hasattr(self, 'final_drive_logged'):
            delattr(self, 'final_drive_logged')

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