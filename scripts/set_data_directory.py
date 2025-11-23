#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class SetDataDirectoryPublisher(Node):
    def __init__(self, directory):
        super().__init__('set_data_directory_pub')
        self.publisher_ = self.create_publisher(String, '/video_recorder/data_directory', 1)
        msg = String()
        msg.data = directory
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published data directory: {directory}')
        rclpy.shutdown()

def main():
    if len(sys.argv) < 2:
        print("Usage: set_data_directory.py <directory_path>")
        sys.exit(1)
    directory = sys.argv[1]
    rclpy.init()
    SetDataDirectoryPublisher(directory)

if __name__ == '__main__':
    main()
