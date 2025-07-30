#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

def main(args=None):
    rclpy.init(args=args)
    node = Node('square_test_publisher')
    
    # Create a publisher on the /path topic
    path_publisher = node.create_publisher(String, '/path', 10)
    
    # Give the publisher a moment to establish connection
    time.sleep(1)
    
    # Define the command for a 0.5-meter square
    # The format is x,y;x,y;...
    path_command = "0.5,0.0;0.5,0.5;0.0,0.5;0.0,0.0"
    
    msg = String()
    msg.data = path_command
    
    path_publisher.publish(msg)
    
    node.get_logger().info(f'Published square path command: "{path_command}"')
    
    # Allow a moment for the message to be sent before shutting down
    time.sleep(1)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 