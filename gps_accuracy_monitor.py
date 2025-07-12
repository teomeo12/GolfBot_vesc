#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import math
import time

class GPSAccuracyMonitor(Node):
    def __init__(self):
        super().__init__('gps_accuracy_monitor')
        
        # Subscribe to GPS fix topic
        self.subscription = self.create_subscription(
            NavSatFix,
            '/ublox_gps_node/fix',  # Standard topic name
            self.gps_callback,
            10)
        
        self.first_fix = None
        self.current_fix = None
        self.last_position = None
        self.start_time = time.time()
        
        self.get_logger().info("🛰️  GPS Accuracy Monitor Started")
        self.get_logger().info("📍 Waiting for GPS fix...")

    def gps_callback(self, msg):
        self.current_fix = msg
        
        # Calculate accuracy from covariance
        x_variance = msg.position_covariance[0]  # X variance (meters²)
        y_variance = msg.position_covariance[4]  # Y variance (meters²)
        
        x_accuracy = math.sqrt(x_variance) if x_variance > 0 else 0
        y_accuracy = math.sqrt(y_variance) if y_variance > 0 else 0
        horizontal_accuracy = math.sqrt(x_variance + y_variance) if (x_variance > 0 and y_variance > 0) else 0
        
        # Store first fix as reference
        if self.first_fix is None:
            self.first_fix = msg
            self.get_logger().info(f"📍 Initial GPS Fix Acquired!")
            self.get_logger().info(f"   Latitude:  {msg.latitude:.8f}°")
            self.get_logger().info(f"   Longitude: {msg.longitude:.8f}°")
            self.get_logger().info(f"   Altitude:  {msg.altitude:.3f} m")
            self.get_logger().info(f"   Accuracy:  {horizontal_accuracy:.3f} m (±{x_accuracy:.3f}m X, ±{y_accuracy:.3f}m Y)")
            
            # Evaluate RTK quality
            if horizontal_accuracy < 0.05:
                self.get_logger().info("🎯 EXCELLENT! RTK Fixed - Centimeter accuracy")
            elif horizontal_accuracy < 0.5:
                self.get_logger().info("✅ GOOD! RTK Float - Sub-meter accuracy") 
            elif horizontal_accuracy < 2.0:
                self.get_logger().info("⚠️  GPS Fix - Meter-level accuracy")
            else:
                self.get_logger().info("❌ Poor GPS accuracy - Check RTK connection")
                
            return
        
        # Calculate distance moved from initial position
        distance = self.calculate_distance(
            self.first_fix.latitude, self.first_fix.longitude,
            msg.latitude, msg.longitude
        )
        
        # Only report if moved significantly (more than accuracy threshold)
        if distance > horizontal_accuracy * 2:  # Move more than 2x the accuracy
            self.get_logger().info(f"📏 Moved {distance:.3f}m from start (accuracy: ±{horizontal_accuracy:.3f}m)")
            
            # If we have a previous position, calculate incremental movement
            if self.last_position is not None:
                incremental_distance = self.calculate_distance(
                    self.last_position.latitude, self.last_position.longitude,
                    msg.latitude, msg.longitude
                )
                if incremental_distance > 0.1:  # 10cm threshold
                    self.get_logger().info(f"   Step: {incremental_distance:.3f}m")
            
            self.last_position = msg
        
        # Periodic status update every 10 seconds
        current_time = time.time()
        if current_time - self.start_time > 10:
            self.get_logger().info(f"📊 Current accuracy: ±{horizontal_accuracy:.3f}m | Total moved: {distance:.3f}m")
            self.start_time = current_time

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance in meters between two GPS points using Haversine formula"""
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
    
    monitor = GPSAccuracyMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info("🛑 GPS Monitor stopped")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 