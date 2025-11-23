#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class UnsetRecordPublisher(Node):
    def __init__(self, pipeline_name):
        super().__init__('unset_record_publisher')
        self.publisher_ = self.create_publisher(String, 'unset_record', 10)
        msg = String()
        msg.data = pipeline_name
        self.get_logger().info(f'Publishing to unset_record: {pipeline_name}')
        self.publisher_.publish(msg)
        rclpy.shutdown()

def main_unset_recording():
    rclpy.init()
    if len(sys.argv) < 2:
        print('Usage: unset_record_pub.py <pipeline_name>')
        sys.exit(1)
    pipeline_name = sys.argv[1]
    node = UnsetRecordPublisher(pipeline_name)
    rclpy.spin(node)

if __name__ == '__main__':
    main_unset_recording()
