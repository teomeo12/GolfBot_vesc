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

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # --- Parameters ---
        self.declare_parameter('wheel_base', 0.58)  # Distance between wheels in meters
        self.declare_parameter('wheel_radius', 0.0825) # Wheel radius in meters (0.165 / 2.0)
        self.declare_parameter('vesc_pole_pairs', 7) # Number of pole pairs in your motor
        self.declare_parameter('vesc_erpm_to_rpm', 0.0) # Calculated later
        
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.vesc_pole_pairs = self.get_parameter('vesc_pole_pairs').get_parameter_value().integer_value
        
        # VESC tachometer measures electrical rotations. We need to convert to mechanical wheel rotations.
        # The VESC's 'tachometer_abs' seems to be 3 * pole_pairs * mechanical_revolutions.
        self.counts_per_revolution = 3 * self.vesc_pole_pairs 
        self.distance_per_count = (2 * math.pi * self.wheel_radius) / self.counts_per_revolution

        # --- State Variables ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_left_counts = None
        self.last_right_counts = None
        
        # --- ROS2 Publishers and Subscribers ---
        qos_profile = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos_profile)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

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
        self.timer = self.create_timer(0.02, self.publish_odometry) # 50 Hz

        self.current_left_counts = None
        self.current_right_counts = None

        self.get_logger().info('Odometry Node Ready.')

    def left_vesc_callback(self, msg):
        # We use displacement here, as it appears to be the most reliable tachometer value
        self.current_left_counts = msg.state.displacement
        if self.last_left_counts is None:
            self.last_left_counts = self.current_left_counts

    def right_vesc_callback(self, msg):
        self.current_right_counts = msg.state.displacement
        if self.last_right_counts is None:
            self.last_right_counts = self.current_right_counts

    def publish_odometry(self):
        # Ensure we have received at least one message from each VESC
        if self.current_left_counts is None or self.current_right_counts is None:
            return

        current_time = self.get_clock().now().to_msg()
        
        # Calculate distance traveled by each wheel
        delta_left_counts = self.current_left_counts - self.last_left_counts
        delta_right_counts = self.current_right_counts - self.last_right_counts
        
        # The VESC encoder counts seem to be inverted on one side. This is a common issue.
        # We assume the right wheel's encoder is the one that needs to be flipped.
        dist_left = delta_left_counts * self.distance_per_count
        dist_right = -delta_right_counts * self.distance_per_count

        self.last_left_counts = self.current_left_counts
        self.last_right_counts = self.current_right_counts

        # --- Differential Drive Kinematics ---
        delta_s = (dist_right + dist_left) / 2.0
        delta_theta = (dist_right - dist_left) / self.wheel_base

        self.x += delta_s * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_s * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

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
        
        # We don't have velocity data directly, so we leave it as zero.
        # A more advanced node would calculate this from the time delta.
        odom_msg.twist.twist.linear.x = 0.0 
        odom_msg.twist.twist.angular.z = 0.0
        odom_msg.twist.covariance = [
            999.9, 0.0, 0.0, 0.0, 0.0, 0.0, # LinX
            0.0, 999.9, 0.0, 0.0, 0.0, 0.0, # LinY
            0.0, 0.0, 999.9, 0.0, 0.0, 0.0, # LinZ
            0.0, 0.0, 0.0, 999.9, 0.0, 0.0, # AngX
            0.0, 0.0, 0.0, 0.0, 999.9, 0.0, # AngY
            0.0, 0.0, 0.0, 0.0, 0.0, 999.9  # AngZ
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