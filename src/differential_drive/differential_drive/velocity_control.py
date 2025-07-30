import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class DifferentialDrive(Node):
    def __init__(self):
        super().__init__('differential_drive_controller')

        # Declare ROS2 parameters for topic names
        self.declare_parameter('left_vel_topic', 'motor')
        self.declare_parameter('right_vel_topic', 'motor')

        # Retrieve parameters
        right_wheel_topic = self.get_parameter('right_vel_topic').get_parameter_value().string_value
        left_wheel_topic = self.get_parameter('left_vel_topic').get_parameter_value().string_value

        if right_wheel_topic == left_wheel_topic:
            self.get_logger().error('The wheels of the differential drive have the same ROS2 Topic. Closing..')
            rclpy.shutdown()

        self.cmd_vel_sub_ = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        self.left_wheel_pub_ = self.create_publisher(Float64, left_wheel_topic, 10)
        self.right_wheel_pub_ = self.create_publisher(Float64, right_wheel_topic, 10)

        self.get_logger().info('Simple Velocity Control - Direct ERPM Passthrough')

    def cmd_vel_callback(self, msg: Twist):
        """
        SWAPPED INTERPRETATION: Robot hardware interprets commands backwards
        - linear.x = turning ERPM (wheels opposite directions) 
        - angular.z = forward/backward ERPM (both wheels same direction)
        """
        
        # Prioritize turning over forward motion - but commands are swapped
        if msg.linear.x != 0.0:
            # LINEAR.X now controls turning: wheels move in opposite directions
            right_speed = msg.linear.x
            left_speed = -msg.linear.x
        elif msg.angular.z != 0.0:
            # ANGULAR.Z now controls forward/backward: both wheels same direction
            drive_speed = msg.angular.z
            right_speed = drive_speed
            left_speed = drive_speed
        else:
            # No movement
            left_speed = 0.0
            right_speed = 0.0
            
        # Publish directly to motors
        left_msg = Float64()
        left_msg.data = float(left_speed)
        self.left_wheel_pub_.publish(left_msg)

        right_msg = Float64()
        right_msg.data = float(right_speed)
        self.right_wheel_pub_.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
