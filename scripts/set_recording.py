#!/usr/bin/env python3

# Author(s): Anton Deguet
# Copyright 2025 Johns Hopkins University

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class SetRecordPublisher(Node):
    def __init__(self, pipeline_name):
        super().__init__('set_record_publisher')
        self.publisher_ = self.create_publisher(String, 'set_record', 10)
        msg = String()
        msg.data = pipeline_name
        self.get_logger().info(f'Publishing to set_record: {pipeline_name}')
        self.publisher_.publish(msg)
        rclpy.shutdown()

def main():
    rclpy.init()
    if len(sys.argv) < 2:
        print('Usage: set_record_pub.py <pipeline_name>')
        sys.exit(1)
    pipeline_name = sys.argv[1]
    node = SetRecordPublisher(pipeline_name)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
