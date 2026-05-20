#!/usr/bin/env python3

# Copyright (c) 2024, 2025 Robert Bosch GmbH
#
# See the top-level LICENSE file for licensing terms.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String


class DummyPublisherNode(Node):

    def __init__(self):
        super().__init__('dummy_publisher_node')

        # Publishers
        self.string_pub1 = self.create_publisher(String, '/dummy_string_topic1', 10)
        self.string_pub2 = self.create_publisher(String, '/dummy_string_topic2', 10)
        self.string_pub2 = self.create_publisher(String, '/dummy_string_topic3', 10)
        self.header_pub = self.create_publisher(CameraInfo, '/dummy_header_topic', 10)

        # Timer to publish at 10Hz
        self.timer = self.create_timer(0.1, self.publish_messages)

        self.get_logger().info('DummyPublisherNode has been started.')

    def publish_messages(self):
        # Publish to /string_topic1
        msg1 = String()
        msg1.data = 'Hello from string_topic1'
        self.string_pub1.publish(msg1)

        # Publish to /string_topic2
        msg2 = String()
        msg2.data = 'Hello from string_topic2'
        self.string_pub2.publish(msg2)

        # Publish to /string_topic3
        msg3 = String()
        msg3.data = 'Hello from string_topic3'
        self.string_pub2.publish(msg3)

        # Publish to /header_topic
        header_msg = CameraInfo()
        header_msg.header.stamp = self.get_clock().now().to_msg()
        header_msg.header.frame_id = 'dummy_frame'
        self.header_pub.publish(header_msg)

        self.get_logger().debug('Published dummy messages to all topics.')


def main(args=None):
    rclpy.init(args=args)
    node = DummyPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down DummyPublisherNode...')


if __name__ == '__main__':
    main()
