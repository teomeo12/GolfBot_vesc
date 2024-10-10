import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class SonyGamepadNode(Node):
    def __init__(self):
        super().__init__('sony_gamepad_node')

        # Create a subscriber for the Joy topic
        self.joy_sub_ = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)

        # Gear settings
        self.lin_speeds = [100, 150, 200]
        self.ang_speeds = [300, 400, 500]
        self.current_gear = 0

        # Track button states to handle bouncing
        self.prev_button_4_state = False
        self.prev_button_5_state = False

        # Create a publisher for the cmd_vel topic
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Store the linear and angular velocities from the joystick
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def joy_callback(self, msg):
        # Extract linear and angular components from the joystick axes
        self.linear_velocity = msg.axes[1] * self.lin_speeds[self.current_gear]  # axes[1] controls the linear velocity
        self.angular_velocity = (-1) * msg.axes[3] * self.ang_speeds[self.current_gear]  # axes[3] controls the angular velocity

        # Check button 4 for decreasing gear
        button_4_pressed = msg.buttons[4] == 1
        if button_4_pressed and not self.prev_button_4_state:
            if self.current_gear > 0:
                self.current_gear -= 1
                self.get_logger().info(f'Gear decreased to: {self.current_gear}')

        # Update previous button 4 state
        self.prev_button_4_state = button_4_pressed

        # Check button 5 for increasing gear
        button_5_pressed = msg.buttons[5] == 1
        if button_5_pressed and not self.prev_button_5_state:
            if self.current_gear < len(self.lin_speeds) - 1:
                self.current_gear += 1
                self.get_logger().info(f'Gear increased to: {self.current_gear}')

        # Update previous button 5 state
        self.prev_button_5_state = button_5_pressed

        # Create a Twist message
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_velocity
        twist_msg.angular.z = self.angular_velocity

        # Publish the Twist message to /cmd_vel
        self.cmd_vel_pub_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SonyGamepadNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
