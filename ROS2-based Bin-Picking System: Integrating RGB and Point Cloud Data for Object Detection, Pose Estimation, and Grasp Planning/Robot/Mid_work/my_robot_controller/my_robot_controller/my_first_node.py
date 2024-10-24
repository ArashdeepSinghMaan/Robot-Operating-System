#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        # Initialize the node with a specific name
        super().__init__('my_first_node')
        #self.get_logger().info("Hello, ROS 2!")
        self.counter_=0
        # Optionally, you can add a timer to demonstrate a periodic task
        self.create_timer(1.0, self.timer_callback)  # Call the timer_callback every 1 second

    def timer_callback(self):
        # This method can be used to perform periodic tasks
        self.get_logger().info("Timer callback executed.")
        self.counter_+=1
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 system, allowing command-line arguments
    node = MyNode()  # Create an instance of MyNode without requiring an argument
    rclpy.spin(node)  # Keep the node running to process callbacks
    node.destroy_node()  # Clean up the node after it's done
    rclpy.shutdown()  # Shutdown the ROS 2 system

if __name__ == '__main__':
    main()

