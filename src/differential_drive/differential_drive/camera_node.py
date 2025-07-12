#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs
import math

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # ROS Publishers
        self.color_pub = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        self.imu_pub = self.create_publisher(Imu, 'camera/imu', 10)
        self.cv_info_pub = self.create_publisher(String, 'camera/cv_info', 10)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Camera parameters (from your working script)
        self.FRAME_WIDTH = 640
        self.FRAME_HEIGHT = 480
        
        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Configure camera streams (same as your working script)
        self.config.enable_stream(rs.stream.depth, self.FRAME_WIDTH, self.FRAME_HEIGHT, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, self.FRAME_WIDTH, self.FRAME_HEIGHT, rs.format.bgr8, 30)
        
        # TODO: Enable IMU streams later (currently causing conflicts)
        # self.config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)  # 250 Hz
        # self.config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)   # 200 Hz
        
        try:
            self.get_logger().info("🎥 Starting RealSense D435i Camera...")
            self.profile = self.pipeline.start(self.config)
            
            # Create align object (from your working script)
            self.align = rs.align(rs.stream.color)
            
            # Get camera FoV (from your working script)
            intrinsics = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
            fov = rs.rs2_fov(intrinsics)
            self.horizontal_fov = fov[0]
            self.vertical_fov = fov[1]
            
            self.get_logger().info("📷 Camera initialized successfully!")
            self.get_logger().info(f"   📐 Horizontal FoV: {self.horizontal_fov:.2f}°")
            self.get_logger().info(f"   📐 Vertical FoV: {self.vertical_fov:.2f}°")
            self.get_logger().info("   🧭 IMU: Disabled for now (will add later)")
            
            # Timer for camera processing
            self.timer = self.create_timer(0.1, self.process_frames)  # 10 Hz
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize camera: {e}")
            
    def process_frames(self):
        """Process camera frames and IMU data"""
        try:
            # Get frames (same as your working script)
            frames = self.pipeline.wait_for_frames()
            
            # TODO: Process IMU data later
            # self.process_imu_data(frames)
            
            # Process camera data (your existing logic)
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                return
                
            # Convert to numpy (same as your script)
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # Process image with your CV logic
            processed_image = self.process_cv_analysis(color_image, depth_frame)
            
            # Publish images
            self.publish_images(processed_image, depth_image)
            
            # Display the image window (like your original script)
            cv2.imshow("RealSense Camera - ROS Node", processed_image)
            cv2.waitKey(1)  # Non-blocking, allows ROS to continue
            
        except Exception as e:
            self.get_logger().error(f"Error processing frames: {e}")
            
    def process_imu_data(self, frames):
        """Process IMU data from the camera"""
        try:
            # Check for accelerometer data
            if frames.first_or_default(rs.stream.accel):
                accel_frame = frames.first_or_default(rs.stream.accel)
                accel_data = accel_frame.as_motion_frame().get_motion_data()
                
                # Check for gyroscope data
                if frames.first_or_default(rs.stream.gyro):
                    gyro_frame = frames.first_or_default(rs.stream.gyro)
                    gyro_data = gyro_frame.as_motion_frame().get_motion_data()
                    
                    # Create IMU message
                    imu_msg = Imu()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = 'camera_imu_frame'
                    
                    # Linear acceleration (m/s²)
                    imu_msg.linear_acceleration.x = accel_data.x
                    imu_msg.linear_acceleration.y = accel_data.y
                    imu_msg.linear_acceleration.z = accel_data.z
                    
                    # Angular velocity (rad/s)
                    imu_msg.angular_velocity.x = gyro_data.x
                    imu_msg.angular_velocity.y = gyro_data.y
                    imu_msg.angular_velocity.z = gyro_data.z
                    
                    # Publish IMU data
                    self.imu_pub.publish(imu_msg)
                    
        except Exception as e:
            # IMU data might not be available every frame, that's okay
            pass
            
    def process_cv_analysis(self, color_image, depth_frame):
        """Your original CV processing logic"""
        height, width, _ = color_image.shape
        center_x, center_y = width // 2, height // 2
        
        # Get distance to center (same as your script)
        distance_m = depth_frame.get_distance(center_x, center_y)
        
        # Calculate real-world dimensions (same as your script)
        view_width_cm = 0
        view_height_cm = 0
        
        if distance_m > 0:
            view_width_cm = 2 * (distance_m * 100) * math.tan(math.radians(self.horizontal_fov / 2))
            view_height_cm = 2 * (distance_m * 100) * math.tan(math.radians(self.vertical_fov / 2))
        
        # Create display image (same as your script)
        display_image = color_image.copy()
        
        # Draw crosshair (same as your script)
        cv2.line(display_image, (center_x, 0), (center_x, height), (0, 0, 255), 2)
        cv2.line(display_image, (0, center_y), (width, center_y), (0, 0, 255), 2)
        
        # Create text background (same as your script)
        text_bg = np.zeros_like(display_image, np.uint8)
        cv2.rectangle(text_bg, (0, 0), (450, 100), (0, 0, 0), -1)
        display_image = cv2.addWeighted(display_image, 1.0, text_bg, 0.5, 0)
        
        # Add text overlay (same as your script)
        cv2.putText(display_image, f"Distance to Center: {distance_m * 100:.1f} cm", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(display_image, f"Frame Width: {view_width_cm:.1f} cm", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(display_image, f"Frame Height: {view_height_cm:.1f} cm", 
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Publish CV analysis results
        cv_info_msg = String()
        cv_info_msg.data = f"Distance: {distance_m * 100:.1f}cm | Width: {view_width_cm:.1f}cm | Height: {view_height_cm:.1f}cm"
        self.cv_info_pub.publish(cv_info_msg)
        
        return display_image
        
    def publish_images(self, color_image, depth_image):
        """Publish camera images as ROS topics"""
        try:
            # Convert to ROS messages
            color_msg = self.bridge.cv2_to_imgmsg(color_image, 'bgr8')
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, 'mono16')
            
            # Add timestamps and frame IDs
            stamp = self.get_clock().now().to_msg()
            
            color_msg.header.stamp = stamp
            color_msg.header.frame_id = 'camera_color_frame'
            
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = 'camera_depth_frame'
            
            # Publish
            self.color_pub.publish(color_msg)
            self.depth_pub.publish(depth_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing images: {e}")
            
    def destroy_node(self):
        """Clean shutdown"""
        try:
            self.pipeline.stop()
            cv2.destroyAllWindows()  # Close camera window
            self.get_logger().info("🛑 Camera stopped")
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = CameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Camera Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 