#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StartRecordingPublisher(Node):
    def __init__(self):
        super().__init__('start_recording_pub')
        self.publisher_ = self.create_publisher(String, '/video_recorder/control', 1)
        msg = String()
        msg.data = 'start_recording'
        self.publisher_.publish(msg)
        self.get_logger().info('Published start_recording message')
        rclpy.shutdown()

def main():
    rclpy.init()
    StartRecordingPublisher()

if __name__ == '__main__':
    main()
