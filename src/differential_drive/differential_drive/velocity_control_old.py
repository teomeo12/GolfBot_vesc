import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class DifferentialDrive(Node):
    def __init__(self):
        super().__init__('differential_drive_controller')

        # Declare ROS2 parameters for constants
        self.declare_parameter('wheel_base', 0.58)  # Distance between the wheels
        self.declare_parameter('wheel_radius', 0.165/2.0)  # Radius of the wheels
        self.declare_parameter('left_vel_topic', 'motor')
        self.declare_parameter('right_vel_topic', 'motor')

        # Retrieve parameters
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        right_wheel_topic = self.get_parameter('right_vel_topic').get_parameter_value().string_value
        left_wheel_topic = self.get_parameter('left_vel_topic').get_parameter_value().string_value

        if right_wheel_topic == left_wheel_topic:
            self.get_logger().error('The wheels of the differential drive have the same ROS2 Topic. Closing..')
            rclpy.shutdown()

        # Create a subscriber for velocity commands (linear and angular)
        self.cmd_vel_sub_ = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # Create publishers for motor control (left and right wheels)
        self.left_wheel_pub_ = self.create_publisher(Float64, left_wheel_topic, 10)
        self.right_wheel_pub_ = self.create_publisher(Float64, right_wheel_topic, 10)

        # Variables to store velocity commands
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocity from the message
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        self.update_motor_commands()

    def update_motor_commands(self):
        # Calculate wheel velocities based on differential drive kinematics
        left_wheel_velocity = (self.linear_velocity - (self.angular_velocity * self.wheel_base / 2)) / self.wheel_radius
        right_wheel_velocity = (self.linear_velocity + (self.angular_velocity * self.wheel_base / 2)) / self.wheel_radius

        # Create Float64 messages
        left_msg = Float64()
        right_msg = Float64()
        left_msg.data = left_wheel_velocity
        right_msg.data = right_wheel_velocity

        # Publish motor commands
        self.left_wheel_pub_.publish(left_msg)
        self.right_wheel_pub_.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()