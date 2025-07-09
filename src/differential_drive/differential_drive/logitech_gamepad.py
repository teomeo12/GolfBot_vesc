import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class LogitechGamepadNode(Node):
    def __init__(self):
        super().__init__('logitech_gamepad_node')

        # Create a subscriber for the Joy topic
        self.joy_sub_ = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)

        # Gear settings
        self.lin_speeds = [100, 150, 200]  # Linear speed settings
        self.ang_speeds = [300, 400, 500]  # Angular speed settings
        self.current_gear = 0

        # Track button states to handle bouncing
        self.prev_lb_state = False  # LB button (button 4)
        self.prev_rb_state = False  # RB button (button 5)

        # Create a publisher for the cmd_vel topic
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Store the linear and angular velocities from the joystick
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Deadzone for stick inputs to prevent drift
        self.deadzone = 0.1

        self.get_logger().info('Logitech F710 Gamepad Node Started')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left Stick (Up/Down): Forward/Backward movement')
        self.get_logger().info('  Right Stick (Left/Right): Turn left/right')
        self.get_logger().info('  LB Button: Decrease gear')
        self.get_logger().info('  RB Button: Increase gear')
        self.get_logger().info(f'  Current gear: {self.current_gear} (Speed: {self.lin_speeds[self.current_gear]})')

    def apply_deadzone(self, value):
        """Apply deadzone to joystick input to prevent drift"""
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def joy_callback(self, msg):
        # Check if we have enough axes and buttons
        if len(msg.axes) < 3 or len(msg.buttons) < 6:
            self.get_logger().warn('Insufficient axes or buttons from joystick')
            return

        # Extract linear and angular components from the joystick axes
        # Logitech F710 in XInput mode:
        # axes[1] = Left stick vertical (forward/backward)
        # axes[3] = Right stick horizontal (left/right turning)
        raw_linear = self.apply_deadzone(msg.axes[1])
        raw_angular = self.apply_deadzone(msg.axes[3])
        
        self.linear_velocity = raw_linear * self.lin_speeds[self.current_gear]
        self.angular_velocity = (-1) * raw_angular * self.ang_speeds[self.current_gear]  # Invert for intuitive turning

        # Check LB button (button 4) for decreasing gear
        lb_pressed = msg.buttons[4] == 1
        if lb_pressed and not self.prev_lb_state:
            if self.current_gear > 0:
                self.current_gear -= 1
                self.get_logger().info(f'Gear decreased to: {self.current_gear} (Speed: {self.lin_speeds[self.current_gear]})')

        # Update previous LB button state
        self.prev_lb_state = lb_pressed

        # Check RB button (button 5) for increasing gear
        rb_pressed = msg.buttons[5] == 1
        if rb_pressed and not self.prev_rb_state:
            if self.current_gear < len(self.lin_speeds) - 1:
                self.current_gear += 1
                self.get_logger().info(f'Gear increased to: {self.current_gear} (Speed: {self.lin_speeds[self.current_gear]})')

        # Update previous RB button state
        self.prev_rb_state = rb_pressed

        # Create a Twist message
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_velocity
        twist_msg.angular.z = self.angular_velocity

        # Publish the Twist message to /cmd_vel
        self.cmd_vel_pub_.publish(twist_msg)

        # Log current velocities (optional, can be commented out to reduce spam)
        # if abs(self.linear_velocity) > 0 or abs(self.angular_velocity) > 0:
        #     self.get_logger().info(f'Linear: {self.linear_velocity:.1f}, Angular: {self.angular_velocity:.1f}')

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