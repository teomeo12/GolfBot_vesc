import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class LogitechGamepadNode(Node):
    def __init__(self):
        super().__init__('logitech_gamepad_node')

        # Create a subscriber for the Joy topic
        self.joy_sub_ = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)

        # Single speed setting (no gears needed)
        self.lin_speed = 550  # Linear speed setting
        self.ang_speed = 550  # Angular speed setting

        # Track button states to handle bouncing
        self.prev_y_state = False   # Y button (button 3)
        self.prev_x_state = False   # X button (button 2)

        # Create publishers
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.autonomous_mode_pub_ = self.create_publisher(Bool, '/is_autonomous_mode', 10)

        # Store the linear and angular velocities from the joystick
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Deadzone for stick inputs to prevent drift
        self.deadzone = 0.1

        self.get_logger().info('Logitech F710 Gamepad Node Started (Single Speed)')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left Stick (Up/Down): Forward/Backward movement')
        self.get_logger().info('  Right Stick (Left/Right): Turn left/right')
        self.get_logger().info('  Y Button: Enable autonomous mode')
        self.get_logger().info('  X Button: Disable autonomous mode')
        self.get_logger().info(f'  Speed Settings - Linear: {self.lin_speed}, Angular: {self.ang_speed}')

    def apply_deadzone(self, value):
        """Apply deadzone to joystick input to prevent drift"""
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def joy_callback(self, msg):
        # Check if we have enough axes and buttons
        if len(msg.axes) < 4 or len(msg.buttons) < 6:
            self.get_logger().warn('Insufficient axes or buttons from joystick')
            return

        # Fix the axis assignments - they were backwards!
        # axes[1] = Left stick vertical (forward/backward) - CORRECT BEHAVIOR
        # axes[3] = Right stick horizontal (left/right turning) - CORRECT BEHAVIOR  
        raw_linear = self.apply_deadzone(msg.axes[1])   # Changed back to axes[1] for forward/backward
        raw_angular = self.apply_deadzone(msg.axes[3])  # Changed back to axes[3] for left/right
        
        self.linear_velocity = raw_linear * self.lin_speed
        self.angular_velocity = raw_angular * self.ang_speed  # Removed inversion for correct turning

        # Check Y button (button 3) for autonomous mode activation
        y_pressed = msg.buttons[3] == 1
        if y_pressed and not self.prev_y_state:
            autonomous_msg = Bool()
            autonomous_msg.data = True
            self.autonomous_mode_pub_.publish(autonomous_msg)
            self.get_logger().info('🤖 Y BUTTON: Autonomous mode ACTIVATED')

        self.prev_y_state = y_pressed

        # Check X button (button 2) for autonomous mode deactivation
        x_pressed = msg.buttons[2] == 1
        if x_pressed and not self.prev_x_state:
            autonomous_msg = Bool()
            autonomous_msg.data = False
            self.autonomous_mode_pub_.publish(autonomous_msg)
            self.get_logger().info('🛑 X BUTTON: Autonomous mode DEACTIVATED')

        self.prev_x_state = x_pressed

        # Create a Twist message
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_velocity
        twist_msg.angular.z = self.angular_velocity
        self.cmd_vel_pub_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LogitechGamepadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()