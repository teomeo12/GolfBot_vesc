#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time
from enum import Enum

class AlignmentState(Enum):
    IDLE = 1
    TURNING = 2
    ADJUSTING_DISTANCE = 3
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
        
        # Camera parameters (RealSense D435i defaults)
        self.camera_fx = 615.0  # Focal length X
        self.camera_fy = 615.0  # Focal length Y
        self.camera_cx = 320.0  # Principal point X
        self.camera_cy = 240.0  # Principal point Y
        self.center_threshold = 50  # Pixels from center to consider aligned
        
        # Physical measurements (in meters)
        self.dispenser_offset = 0.90  # Dispenser is 90cm behind camera
        
        # --- Simple Fixed Speed Control Parameters ---
        # Based on manual mode data: angular ~82, linear speeds that work
        self.turn_speed = 100.0           # Fixed angular speed for turning
        self.drive_speed = 80.0          # Fixed linear speed for alignment
        self.final_drive_speed = 80.0    # Fixed speed for final positioning drive
        
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
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dispense_pub = self.create_publisher(String, '/dispense_sand', 10)
        self.autonomous_mode_pub = self.create_publisher(Bool, '/is_autonomous_mode', 10)
        
        # Control timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Align and Repair Node Ready')
        
    def autonomous_mode_callback(self, msg):
        """Handle autonomous mode activation/deactivation"""
        if msg.data and not self.autonomous_mode:
            self.autonomous_mode = True
            self.get_logger().info('ENTERING AUTONOMOUS MODE: Starting sequence.')
            self.state = AlignmentState.TURNING
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
            
        if self.state == AlignmentState.TURNING:
            self.handle_turning()
        elif self.state == AlignmentState.ADJUSTING_DISTANCE:
            self.handle_adjusting_distance()
        elif self.state == AlignmentState.WAITING:
            self.handle_waiting()
        elif self.state == AlignmentState.DRIVING:
            self.handle_driving()
        elif self.state == AlignmentState.DISPENSING:
            self.handle_dispensing()
            
    def handle_turning(self):
        """Turn robot until divot is horizontally centered"""
        if not hasattr(self, 'turning_logged'):
            self.get_logger().info('STATE: TURNING - Aligning with divot.')
            self.turning_logged = True

        if self.latest_detection_info is None:
            self.stop_robot()
            return

        pixel_error = self.latest_detection_info['center_x'] - self.camera_cx
        
        if abs(pixel_error) < self.center_threshold:
            self.get_logger().info('Horizontal alignment complete.')
            self.stop_robot()
            self.state = AlignmentState.ADJUSTING_DISTANCE
            return

        twist = Twist()
        # To turn right (positive pixel_error), we need a negative angular.z
        twist.angular.z = -self.turn_speed if pixel_error > 0 else self.turn_speed
        self.cmd_vel_pub.publish(twist)

    def handle_adjusting_distance(self):
        """Drive robot forward/backward until divot is vertically centered."""
        if not hasattr(self, 'adjusting_logged'):
            self.get_logger().info('STATE: ADJUSTING_DISTANCE - Aligning vertically.')
            self.adjusting_logged = True

        if self.latest_detection_info is None:
            self.stop_robot()
            return

        pixel_error_y = self.latest_detection_info['center_y'] - self.camera_cy

        if abs(pixel_error_y) < self.center_threshold:
            self.get_logger().info('Vertical alignment complete.')
            self.stop_robot()
            self.state = AlignmentState.WAITING
            self.wait_start_time = time.time()
            return

        twist = Twist()
        # If divot is below center (positive error), move forward (-speed).
        # If divot is above center (negative error), move backward (+speed).
        twist.linear.x = -self.drive_speed if pixel_error_y > 0 else self.drive_speed
        self.cmd_vel_pub.publish(twist)
        
    def handle_waiting(self):
        """Wait 5 seconds then lock target position"""
        if not hasattr(self, 'waiting_logged'):
            self.get_logger().info('STATE: WAITING - Stabilizing for 5 seconds.')
            self.waiting_logged = True

        elapsed = time.time() - self.wait_start_time
        
        if elapsed >= 5.0:
            self.get_logger().info('Wait complete. Attempting to lock target...')
            if self.latest_detection_info is not None:
                divot_pos = self.get_divot_world_position(
                    self.latest_detection_info['center_x'], 
                    self.latest_detection_info['center_y']
                )
                if divot_pos is not None:
                    self.locked_target_position = divot_pos
                    self.target_locked = True
                    self.get_logger().info(f"TARGET LOCKED: Position (forward_dist={divot_pos['forward_dist']:.2f}m, lateral_offset={divot_pos['lateral_offset']:.2f}m)")
                    self.state = AlignmentState.DRIVING
                else:
                    self.get_logger().error('LOCK FAILED: Could not get valid world position (invalid depth?). Resetting.')
                    self.reset_sequence() # Reset if we can't get a position
            else:
                self.get_logger().error('LOCK FAILED: No divot detected after wait. Resetting.')
                self.reset_sequence() # Reset if no detection
        
    def handle_driving(self):
        """Drive to locked position accounting for dispenser offset. This is an open-loop, distance-based controller."""
        
        # On first entry, calculate the total distance we need to travel.
        if not hasattr(self, 'distance_to_drive_total'):
            self.get_logger().info('STATE: DRIVING - Calculating travel distance.')
            
            if not self.target_locked:
                self.get_logger().error("DRIVE FAILED: Target not locked. Resetting.")
                self.stop_robot()
                self.reset_sequence()
                return

            # The total distance is the divot's forward distance PLUS the dispenser's offset.
            self.distance_to_drive_total = self.locked_target_position['forward_dist'] + self.dispenser_offset
            self.distance_driven = 0.0 # Reset our driven distance tracker
            
            # Assumed speed (m/s) for the final, fixed-speed drive.
            # You may need to calibrate this value for accuracy.
            self.assumed_speed_m_per_s = 0.25 

            self.get_logger().info(f"Required travel distance: {self.distance_to_drive_total:.2f}m")
            if self.distance_to_drive_total < 0.05:
                self.get_logger().info("Already at target, skipping drive.")
                self.state = AlignmentState.DISPENSING
                self.dispense_start_time = time.time()
                delattr(self, 'distance_to_drive_total')
                return

        # Estimate distance covered in the last time slice (0.1s) and accumulate it.
        # This is an open-loop estimation.
        self.distance_driven += self.assumed_speed_m_per_s * 0.1 # 0.1 is the control_timer period

        if self.distance_driven >= self.distance_to_drive_total:
            self.get_logger().info(f"Drive complete. (Estimated distance driven: {self.distance_driven:.2f}m)")
            self.stop_robot()
            self.state = AlignmentState.DISPENSING
            self.dispense_start_time = time.time()
            # Clear the state for the next run
            delattr(self, 'distance_to_drive_total')
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

        # Reset logging flags
        if hasattr(self, 'turning_logged'): delattr(self, 'turning_logged')
        if hasattr(self, 'adjusting_logged'): delattr(self, 'adjusting_logged')
        if hasattr(self, 'waiting_logged'): delattr(self, 'waiting_logged')
        if hasattr(self, 'driving_logged'): delattr(self, 'driving_logged')
        if hasattr(self, 'dispensing_logged'): delattr(self, 'dispensing_logged')
        
        if hasattr(self, 'dispense_started'):
            delattr(self, 'dispense_started')
        if hasattr(self, 'dispense_stopped'):
            delattr(self, 'dispense_stopped')

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