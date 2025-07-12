#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import math
import time

class GPSMonitorNode(Node):
    def __init__(self):
        super().__init__('gps_monitor_node')
        
        # Subscribe to GPS fix topic
        self.subscription = self.create_subscription(
            NavSatFix,
            '/ublox_gps_node/fix',  # GPS topic from launch file
            self.gps_callback,
            10)
        
        # Publisher for GPS status messages
        self.status_pub = self.create_publisher(String, 'gps_status', 10)
        
        # Timer for periodic status updates
        self.timer = self.create_timer(5.0, self.status_timer_callback)
        
        self.first_fix = None
        self.current_fix = None
        self.last_position = None
        self.start_time = time.time()
        self.last_accuracy = 0.0
        self.total_distance = 0.0
        
        self.get_logger().info("🛰️  GPS Monitor Node Started")
        self.get_logger().info("📍 Waiting for GPS fix from /ublox_gps_node/fix topic...")

    def gps_callback(self, msg):
        self.current_fix = msg
        
        # Calculate accuracy from covariance
        x_variance = msg.position_covariance[0]  # X variance (meters²)
        y_variance = msg.position_covariance[4]  # Y variance (meters²)
        
        x_accuracy = math.sqrt(x_variance) if x_variance > 0 else 0
        y_accuracy = math.sqrt(y_variance) if y_variance > 0 else 0
        horizontal_accuracy = math.sqrt(x_variance + y_variance) if (x_variance > 0 and y_variance > 0) else 0
        self.last_accuracy = horizontal_accuracy
        
        # Store first fix as reference
        if self.first_fix is None:
            self.first_fix = msg
            self.get_logger().info(f"📍 Initial GPS Fix Acquired!")
            self.get_logger().info(f"   Lat: {msg.latitude:.8f}° | Lon: {msg.longitude:.8f}°")
            self.get_logger().info(f"   Alt: {msg.altitude:.3f}m | Accuracy: ±{horizontal_accuracy:.3f}m")
            
            # Evaluate RTK quality
            status_msg = String()
            if horizontal_accuracy < 0.05:
                rtk_status = "🎯 RTK FIXED - Centimeter accuracy"
                self.get_logger().info(rtk_status)
            elif horizontal_accuracy < 0.5:
                rtk_status = "✅ RTK FLOAT - Sub-meter accuracy" 
                self.get_logger().info(rtk_status)
            elif horizontal_accuracy < 2.0:
                rtk_status = "⚠️  GPS STANDARD - Meter accuracy"
                self.get_logger().info(rtk_status)
            else:
                rtk_status = "❌ POOR GPS - Check connections"
                self.get_logger().info(rtk_status)
                
            status_msg.data = f"INIT: {rtk_status} | Accuracy: ±{horizontal_accuracy:.3f}m"
            self.status_pub.publish(status_msg)
            return
        
        # Calculate distance moved from initial position
        distance = self.calculate_distance(
            self.first_fix.latitude, self.first_fix.longitude,
            msg.latitude, msg.longitude
        )
        self.total_distance = distance
        
        # Only report significant movement
        if distance > horizontal_accuracy * 2:  # Move more than 2x the accuracy
            # Publish movement status (no console logging to keep launch terminal clean)
            status_msg = String()
            status_msg.data = f"MOVED: {distance:.3f}m | Accuracy: ±{horizontal_accuracy:.3f}m"
            self.status_pub.publish(status_msg)

    def status_timer_callback(self):
        """Periodic status update"""
        if self.current_fix is not None:
            status_msg = String()
            status_msg.data = f"STATUS: Accuracy ±{self.last_accuracy:.3f}m | Total moved: {self.total_distance:.3f}m"
            self.status_pub.publish(status_msg)

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance in meters between two GPS points"""
        # Convert to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        
        # Haversine formula
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        
        # Earth's radius in meters
        r = 6371000
        distance = r * c
        return distance

def main(args=None):
    rclpy.init(args=args)
    
    monitor = GPSMonitorNode()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info("🛑 GPS Monitor stopped")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 