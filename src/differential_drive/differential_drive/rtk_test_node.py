#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Joy, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import time

class RTKTestNode(Node):
    def __init__(self):
        super().__init__('rtk_test_node')
        
        # Subscriptions
        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.gps_callback, 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.results_pub = self.create_publisher(String, 'rtk_test_results', 10)
        self.status_pub = self.create_publisher(String, 'rtk_status', 10)
        
        # State variables
        self.current_lat = None
        self.current_lon = None
        self.current_yaw = None
        self.position_covariance = None # Added back
        self.start_pos = None
        self.target_heading = None
        self.test_in_progress = False
        self.prev_a_state = False
        
        # Parameters
        self.drive_speed = 550.0
        self.heading_kp = 2.5  # Proportional gain for heading correction

        # Control loop timer
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('RTK Test Node (Heading Lock) Initialized')
        self.get_logger().info('Press A Button: Start 1-Meter Forward Test')
        self.get_logger().info('Press B Button: Stop Test')

    def imu_callback(self, msg):
        """Handle IMU data for heading (yaw)"""
        q = msg.orientation
        # Conversion from quaternion to yaw (in radians)
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.position_covariance = msg.position_covariance # Added back

    def joy_callback(self, msg):
        if len(msg.buttons) < 2: return
        
        a_pressed = msg.buttons[0] == 1
        if a_pressed and not self.prev_a_state:
            self.start_rtk_test()
        self.prev_a_state = a_pressed
            
        b_pressed = msg.buttons[1] == 1
        if b_pressed:
            self.stop_rtk_test()

    def start_rtk_test(self):
        if self.current_lat is None or self.current_yaw is None:
            self.get_logger().warn("Cannot start test: No GPS or IMU data yet.")
            return
            
        if self.test_in_progress:
            self.get_logger().warn("Test already in progress.")
            return

        self.start_pos = (self.current_lat, self.current_lon)
        self.target_heading = self.current_yaw # Lock to the initial heading
        self.test_in_progress = True

        self.get_logger().info("🚀 STARTING 1-METER FORWARD TEST (with Heading Lock)...")
        self.get_logger().info(f"Start Position: {self.start_pos[0]:.7f}, {self.start_pos[1]:.7f}")
        self.get_logger().info(f"Locked Target Heading: {math.degrees(self.target_heading):.1f}°")
        
        start_result_msg = String()
        start_result_msg.data = f"START | Lat: {self.start_pos[0]:.8f}, Lon: {self.start_pos[1]:.8f}"
        self.results_pub.publish(start_result_msg)

    def stop_rtk_test(self):
        if self.test_in_progress:
            self.test_in_progress = False
            self.send_stop_command()
            self.get_logger().info("🛑 TEST STOPPED BY USER.")

    def control_loop(self):
        if not self.test_in_progress:
            return

        if self.start_pos is None or self.current_lat is None or self.current_yaw is None:
            self.status_pub.publish(String(data="Waiting for GPS and IMU data..."))
            return

        distance_traveled = self.gps_to_meters(self.start_pos[0], self.start_pos[1], self.current_lat, self.current_lon)

        if distance_traveled < 1.0:
            # --- Heading Correction ---
            heading_error = self.target_heading - self.current_yaw
            # Normalize angle error to be within -pi to pi
            while heading_error > math.pi: heading_error -= 2 * math.pi
            while heading_error < -math.pi: heading_error += 2 * math.pi
            
            # Proportional control for steering
            angular_velocity = self.heading_kp * heading_error

            # Calculate current accuracy
            accuracy_cm = 0.0
            if self.position_covariance is not None and len(self.position_covariance) >= 5:
                # Horizontal accuracy = sqrt(var_x + var_y)
                accuracy_cm = math.sqrt(self.position_covariance[0] + self.position_covariance[4]) * 100

            # Publish ongoing status to the status topic
            status_msg = String()
            status_msg.data = (f"Driving... Dist: {distance_traveled:.2f}m | "
                             f"Hdg Err: {math.degrees(heading_error):.1f}° | "
                             f"Accuracy: {accuracy_cm:.1f}cm")
            self.status_pub.publish(status_msg)

            # Drive forward with steering correction
            cmd = Twist()
            cmd.linear.x = self.drive_speed
            cmd.angular.z = angular_velocity
            self.cmd_vel_pub.publish(cmd)
        else:
            # Test finished
            self.test_in_progress = False
            self.send_stop_command()
            
            final_pos = (self.current_lat, self.current_lon)
            final_distance = self.gps_to_meters(self.start_pos[0], self.start_pos[1], final_pos[0], final_pos[1])

            self.get_logger().info("🏁 TEST COMPLETE 🏁")
            
            result_msg = String()
            result_msg.data = (
                f"END   | Lat: {final_pos[0]:.8f}, Lon: {final_pos[1]:.8f} | "
                f"Distance: {final_distance:.3f}m"
            )
            self.results_pub.publish(result_msg)
            self.status_pub.publish(String(data=f"Test Complete. Final Distance: {final_distance:.3f}m"))

    def send_stop_command(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def gps_to_meters(self, lat1, lon1, lat2, lon2):
        R = 6371000
        lat1_rad, lon1_rad = math.radians(lat1), math.radians(lon1)
        lat2_rad, lon2_rad = math.radians(lat2), math.radians(lon2)

        dlon = lon2_rad - lon1_rad
        dlat = lat2_rad - lat1_rad

        a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c
        return distance

def main(args=None):
    rclpy.init(args=args)
    node = RTKTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()