#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math
import time

class RTKTestNode(Node):
    def __init__(self):
        super().__init__('rtk_test_node')
        
        # RTK GPS subscription
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/ublox_gps_node/fix',
            self.gps_callback,
            10)
        
        # Joy subscription for A/B buttons
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        
        # Velocity command publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Current GPS position
        self.current_lat = None
        self.current_lon = None
        self.current_altitude = None
        self.gps_status = None
        self.position_covariance = None
        self.autonomous_mode = False
        
        # Simple navigation parameters
        self.start_lat = None
        self.start_lon = None
        self.current_waypoint = 1  # 1 = forward, 2 = left, 0 = done
        self.waypoint_tolerance = 0.3  # 30cm tolerance
        self.drive_speed = 550.0  # Same as align_and_repair_node
        self.turn_speed = 550.0   # Same as align_and_repair_node
        self.navigation_active = False
        
        # Button state tracking
        self.prev_a_state = False
        self.prev_b_state = False
        
        # Control loop timer
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info('RTK Test Node initialized - Simple 2 waypoint test')
        self.get_logger().info('A Button: Start RTK test (50cm forward, then 50cm left)')
        self.get_logger().info('B Button: Stop RTK test')
        
    def joy_callback(self, msg):
        """Handle A/B button presses"""
        if len(msg.buttons) < 2:
            return
            
        # A button (button 0) - Start RTK test
        a_pressed = msg.buttons[0] == 1
        if a_pressed and not self.prev_a_state:
            self.start_rtk_test()
            
        # B button (button 1) - Stop RTK test  
        b_pressed = msg.buttons[1] == 1
        if b_pressed and not self.prev_b_state:
            self.stop_rtk_test()
            
        self.prev_a_state = a_pressed
        self.prev_b_state = b_pressed
        
    def start_rtk_test(self):
        """Start simple RTK test from current position"""
        if self.current_lat is None or self.current_lon is None:
            self.get_logger().error("No GPS fix - cannot start RTK test!")
            return
            
        # Check accuracy based on covariance instead of status
        accuracy_m = 0.0
        if self.position_covariance is not None and len(self.position_covariance) >= 9:
            accuracy_m = math.sqrt((self.position_covariance[0] + self.position_covariance[4]) / 2)
            
        if accuracy_m < 0.05:
            self.get_logger().info(f"Excellent accuracy ({accuracy_m*100:.1f}cm) - RTK quality test!")
        elif accuracy_m < 0.20:
            self.get_logger().info(f"Good accuracy ({accuracy_m*100:.1f}cm) - suitable for 50cm test!")
        elif accuracy_m < 0.50:
            self.get_logger().warn(f"Moderate accuracy ({accuracy_m*100:.1f}cm) - test may be less precise")
        else:
            self.get_logger().warn(f"Poor accuracy ({accuracy_m*100:.1f}cm) - test results may not be reliable")
            
        self.start_lat = self.current_lat
        self.start_lon = self.current_lon
        self.current_waypoint = 1  # Start with waypoint 1 (forward)
        self.navigation_active = True
        
        self.get_logger().info("🚀 A BUTTON: Starting RTK test!")
        self.get_logger().info(f"Start position: {self.start_lat:.7f}, {self.start_lon:.7f}")
        self.get_logger().info("Step 1: Moving 50cm forward (north)")
        
    def stop_rtk_test(self):
        """Stop RTK test"""
        self.navigation_active = False
        self.send_stop_command()
        self.get_logger().info("🛑 B BUTTON: RTK test stopped!")
        
    def gps_callback(self, msg):
        """Handle GPS data updates"""
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_altitude = msg.altitude
        self.gps_status = msg.status.status
        self.position_covariance = msg.position_covariance
        
        # Calculate accuracy from covariance (sqrt of diagonal elements)
        accuracy_m = 0.0
        if self.position_covariance is not None and len(self.position_covariance) >= 9:
            # Covariance matrix is [xx, xy, xz, yx, yy, yz, zx, zy, zz]
            # Take average of xx and yy (horizontal accuracy)
            accuracy_m = math.sqrt((self.position_covariance[0] + self.position_covariance[4]) / 2)
        
        # Log GPS quality based on accuracy, not status
        if accuracy_m < 0.05:  # < 5cm
            status_text = f"RTK Fixed ({accuracy_m*100:.1f}cm)"
        elif accuracy_m < 0.20:  # < 20cm  
            status_text = f"RTK Float ({accuracy_m*100:.1f}cm)"
        elif accuracy_m < 1.0:   # < 1m
            status_text = f"GPS Fix ({accuracy_m*100:.1f}cm)"
        else:
            status_text = f"Poor GPS ({accuracy_m*100:.1f}cm)"
            
        # Log position occasionally
        if hasattr(self, '_last_log_time'):
            if time.time() - self._last_log_time > 2.0:  # Every 2 seconds
                self.get_logger().info(f"GPS: {status_text} - Lat: {self.current_lat:.7f}, Lon: {self.current_lon:.7f}, Alt: {self.current_altitude:.2f}m")
                self._last_log_time = time.time()
        else:
            self._last_log_time = time.time()
            
            
        
        
    def send_stop_command(self):
        """Send zero velocity command"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        
    def gps_to_meters(self, lat1, lon1, lat2, lon2):
        """Convert GPS coordinates to distance in meters"""
        # Approximate conversion using equirectangular projection
        # Good enough for short distances
        R = 6371000  # Earth radius in meters
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        # Distance calculation
        x = delta_lon * math.cos((lat1_rad + lat2_rad) / 2)
        y = delta_lat
        distance = math.sqrt(x*x + y*y) * R
        
        # Bearing calculation
        bearing = math.atan2(x, y)
        
        return distance, bearing
        
    def control_loop(self):
        """Simple control loop - no proportional control"""
        if not self.navigation_active:
            return
            
        if self.current_lat is None or self.current_lon is None:
            self.get_logger().warn("No GPS data available", throttle_duration_sec=2.0)
            return
            
        # Check GPS accuracy based on covariance
        accuracy_m = 0.0
        if self.position_covariance is not None and len(self.position_covariance) >= 9:
            accuracy_m = math.sqrt((self.position_covariance[0] + self.position_covariance[4]) / 2)
        
        # Only warn if accuracy is very poor (> 2m)
        if accuracy_m > 2.0:
            self.get_logger().warn(f"Very poor GPS accuracy ({accuracy_m*100:.0f}cm) - stopping test", throttle_duration_sec=3.0)
            self.send_stop_command()
            return
            
        if self.current_waypoint == 0:  # Test complete
            self.get_logger().info("🏁 RTK test complete!")
            self.navigation_active = False
            self.send_stop_command()
            return
            
        # Calculate target position based on current waypoint
        if self.current_waypoint == 1:  # Forward 50cm
            lat_per_meter = 1.0 / 111111.0
            target_lat = self.start_lat + (0.5 * lat_per_meter)  # 50cm north
            target_lon = self.start_lon
            description = "50cm Forward"
        else:  # Left 50cm (waypoint 2)
            lat_per_meter = 1.0 / 111111.0
            lon_per_meter = 1.0 / (111111.0 * math.cos(math.radians(self.start_lat)))
            target_lat = self.start_lat + (0.5 * lat_per_meter)  # 50cm north
            target_lon = self.start_lon - (0.5 * lon_per_meter)  # 50cm west
            description = "50cm Left"
            
        # Calculate distance to target
        distance, bearing = self.gps_to_meters(
            self.current_lat, self.current_lon,
            target_lat, target_lon
        )
        
        self.get_logger().info(f"📍 {description} | Distance: {distance*100:.0f}cm", 
                              throttle_duration_sec=1.0)
        
        # Check if we've reached the waypoint
        if distance < self.waypoint_tolerance:
            self.get_logger().info(f"✅ Reached: {description}")
            self.current_waypoint += 1
            
            if self.current_waypoint == 2:
                self.get_logger().info("🎯 Next: 50cm Left")
            elif self.current_waypoint > 2:
                self.get_logger().info("🏁 RTK test complete!")
                self.current_waypoint = 0
                
            self.send_stop_command()
            time.sleep(1.0)  # Pause between waypoints
            return
            
        # Simple movement commands - no proportional control
        cmd = Twist()
        
        # Determine if we need to turn first
        angle_error = bearing
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
            
        # Simple logic: turn first, then drive
        if abs(angle_error) > math.radians(10):  # Need to turn more than 10 degrees
            # Just turn, don't drive
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed if angle_error > 0 else -self.turn_speed
        else:
            # Drive forward, no turning
            cmd.linear.x = self.drive_speed
            cmd.angular.z = 0.0
            
        self.cmd_vel_pub.publish(cmd)
def main(args=None):
    rclpy.init(args=args)
    
    node = RTKTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("RTK Test Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()