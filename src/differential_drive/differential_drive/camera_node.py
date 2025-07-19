#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import pyrealsense2 as rs

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # ROS Publishers for visual data only
        self.color_pub = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        
        self.bridge = CvBridge()
        
        # Camera parameters
        self.FRAME_WIDTH = 640
        self.FRAME_HEIGHT = 480
        
        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Configure camera streams
        self.config.enable_stream(rs.stream.depth, self.FRAME_WIDTH, self.FRAME_HEIGHT, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, self.FRAME_WIDTH, self.FRAME_HEIGHT, rs.format.bgr8, 30)

        try:
            self.get_logger().info("🎥 Starting RealSense D435i Camera...")
            self.profile = self.pipeline.start(self.config)
            self.align = rs.align(rs.stream.color)
            self.get_logger().info("📷 Camera initialized successfully!")
            
            # Timer for frame processing
            self.timer = self.create_timer(1/30.0, self.process_frames)  # ~30 Hz

        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize camera: {e}")
            
    def process_frames(self):
        """Processes and publishes camera frames."""
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            if not frames:
                return

            # Align frames
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                return
                
            # Convert to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # Publish raw images
            self.publish_images(color_image, depth_image)
            
        except Exception as e:
            self.get_logger().error(f"Error processing frames: {e}")

    def publish_images(self, color_image, depth_image):
        """Publish camera images as ROS topics"""
        try:
            stamp = self.get_clock().now().to_msg()
            
            color_msg = self.bridge.cv2_to_imgmsg(color_image, 'bgr8')
            color_msg.header.stamp = stamp
            color_msg.header.frame_id = 'camera_color_frame'
            
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, '16UC1') # Use 16UC1 for depth
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = 'camera_depth_frame'
            
            self.color_pub.publish(color_msg)
            self.depth_pub.publish(depth_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing images: {e}")
            
    def destroy_node(self):
        """Clean shutdown"""
        try:
            self.pipeline.stop()
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