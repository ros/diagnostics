# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.node import Node


class Listener(Node):

    def __init__(self):
        super().__init__('test_listener')
        self.sub = self.create_subscription(
            DiagnosticArray,
            'diagnostics_agg',
            self.test_listener_callback, 10)

    def test_listener_callback(self, msg):
        for status in msg.status:
            self.get_logger().info('%s' % status)


def main(args=None):
    rclpy.init(args=args)
    os.environ['OSPL_VERBOSITY'] = '8'
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{message}'

    node = Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
