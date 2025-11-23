#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StopRecordingPublisher(Node):
    def __init__(self):
        super().__init__('stop_recording_pub')
        self.publisher_ = self.create_publisher(String, '/video_recorder/control', 1)
        msg = String()
        msg.data = 'stop_recording'
        self.publisher_.publish(msg)
        self.get_logger().info('Published stop_recording message')
        rclpy.shutdown()

def main():
    rclpy.init()
    StopRecordingPublisher()

if __name__ == '__main__':
    main()
