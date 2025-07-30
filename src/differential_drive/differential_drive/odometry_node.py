#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from vesc_interfaces.msg import VescStateStamped
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
from std_msgs.msg import String

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # --- Parameters based on direct measurement ---
        self.declare_parameter('wheel_base', 0.57) # Measured distance between wheel ground contacts
        self.declare_parameter('meters_per_unit', 0.0059444) # 0.535 meters / 90 VESC units
        
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.meters_per_unit = self.get_parameter('meters_per_unit').get_parameter_value().double_value

        # --- State Variables ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Renamed for clarity to match VESC sensor data
        self.last_left_displacement = None
        self.last_right_displacement = None
        self.current_left_displacement = None
        self.current_right_displacement = None

        self.last_left_distance_traveled = None
        self.last_right_distance_traveled = None
        self.current_left_distance_traveled = None
        self.current_right_distance_traveled = None
        
        # --- ROS2 Publishers and Subscribers ---
        qos_profile = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos_profile)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.debug_pub = self.create_publisher(String, '/odom/debug_data', 10) # New publisher for debug data

        self.left_vesc_sub = self.create_subscription(
            VescStateStamped,
            '/left_vesc/sensors/core',
            self.left_vesc_callback,
            qos_profile)
        self.right_vesc_sub = self.create_subscription(
            VescStateStamped,
            '/right_vesc/sensors/core',
            self.right_vesc_callback,
            qos_profile)
        
        # Timer for processing data and publishing odometry
        self.timer_period = 0.02  # seconds (for 50 Hz)
        self.timer = self.create_timer(self.timer_period, self.publish_odometry)

        self.get_logger().info('Odometry Node Ready.')

    def left_vesc_callback(self, msg):
        self.current_left_displacement = msg.state.displacement
        self.current_left_distance_traveled = msg.state.distance_traveled
        if self.last_left_displacement is None:
            self.last_left_displacement = self.current_left_displacement
            self.last_left_distance_traveled = self.current_left_distance_traveled

    def right_vesc_callback(self, msg):
        self.current_right_displacement = msg.state.displacement
        self.current_right_distance_traveled = msg.state.distance_traveled
        if self.last_right_displacement is None:
            self.last_right_displacement = self.current_right_displacement
            self.last_right_distance_traveled = self.current_right_distance_traveled

    def publish_odometry(self):
        # Ensure we have received at least one message from each VESC
        if (self.current_left_distance_traveled is None or 
            self.current_right_distance_traveled is None):
            return

        current_time = self.get_clock().now().to_msg()
        
        # --- Calculate true signed distance for each wheel ---
        
        # Left Wheel
        delta_left_displacement = self.current_left_displacement - self.last_left_displacement
        delta_left_distance_traveled = self.current_left_distance_traveled - self.last_left_distance_traveled
        direction_left = math.copysign(1, delta_left_displacement) if delta_left_displacement != 0 else 0.0
        dist_left = direction_left * delta_left_distance_traveled * self.meters_per_unit
        
        # Right Wheel
        delta_right_displacement = self.current_right_displacement - self.last_right_displacement
        delta_right_distance_traveled = self.current_right_distance_traveled - self.last_right_distance_traveled
        direction_right = math.copysign(1, delta_right_displacement) if delta_right_displacement != 0 else 0.0
        # The -1.0 factor corrects for the inverted mounting/reporting of the VESC.
        dist_right = -1.0 * direction_right * delta_right_distance_traveled * self.meters_per_unit

        # Update last known counts for the next cycle
        self.last_left_displacement = self.current_left_displacement
        self.last_right_displacement = self.current_right_displacement
        self.last_left_distance_traveled = self.current_left_distance_traveled
        self.last_right_distance_traveled = self.current_right_distance_traveled

        # --- Differential Drive Kinematics ---
        delta_s = (dist_right + dist_left) / 2.0
        delta_theta = (dist_right - dist_left) / self.wheel_base

        self.x += delta_s * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_s * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        # --- NEW: Calculate Velocities ---
        # Based on the changes over the timer period
        linear_velocity_x = delta_s / self.timer_period
        angular_velocity_z = delta_theta / self.timer_period

        # --- Create and Publish Odometry Message ---
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # --- Publish raw VESC data for debugging ---
        debug_msg = String()
        debug_msg.data = (
            f"Odom_X: {self.x:+.3f}m | "
            f"L_Raw_Displ: {self.current_left_displacement} | "
            f"R_Raw_Displ: {self.current_right_displacement}"
        )
        self.debug_pub.publish(debug_msg)
        
        # Populate covariance matrices. We are confident in our x, y, and yaw
        # position, but not in other values. We reflect that here.
        # A larger number means less confidence.
        odom_msg.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,  # X
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,  # Y
            0.0, 0.0, 999.9, 0.0, 0.0, 0.0, # Z
            0.0, 0.0, 0.0, 999.9, 0.0, 0.0, # Roll
            0.0, 0.0, 0.0, 0.0, 999.9, 0.0, # Pitch
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1    # Yaw
        ]
        
        # We now have velocity data, so we can populate the twist message
        odom_msg.twist.twist.linear.x = linear_velocity_x
        odom_msg.twist.twist.angular.z = angular_velocity_z
        odom_msg.twist.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,  # LinX
            0.0, 999.9, 0.0, 0.0, 0.0, 0.0, # LinY
            0.0, 0.0, 999.9, 0.0, 0.0, 0.0, # LinZ
            0.0, 0.0, 0.0, 999.9, 0.0, 0.0, # AngX
            0.0, 0.0, 0.0, 0.0, 999.9, 0.0, # AngY
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1   # AngZ
        ]
        
        self.odom_pub.publish(odom_msg)

        # --- Broadcast TF Transform ---
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 