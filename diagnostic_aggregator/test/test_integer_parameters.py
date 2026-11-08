# Software License Agreement (BSD License)
#
# Copyright (c) 2026, Tomoya Fujita
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Test that floating point parameters also accept integer-typed YAML values.

"pub_rate: 2" and "timeout: 2" are typed as integer parameters by rclcpp.
Before the fix, rclcpp::Parameter::as_double() threw on them and the
aggregator died during initialization. Now they must be accepted and the
aggregator must produce output.
"""

import tempfile

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import launch
import launch_pytest
import launch_ros
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future


class DiagnosticsTestNode(Node):
    """Publishes a raw diagnostic and waits for the aggregated header."""

    def __init__(self):
        super().__init__(node_name='integer_parameters_test_node')
        self.future = Future()
        self.subscriber = self.create_subscription(
            msg_type=DiagnosticArray,
            topic='/diagnostics_agg',
            callback=self.diagnostics_aggregated_callback,
            qos_profile=10,
        )
        self.publisher = self.create_publisher(
            msg_type=DiagnosticArray, topic='/diagnostics', qos_profile=10
        )
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status.append(
            DiagnosticStatus(name='foo', level=DiagnosticStatus.OK, message='Foo')
        )
        self.publisher.publish(msg)

    def diagnostics_aggregated_callback(self, msg):
        for status in msg.status:
            if status.name == '/robot/agg' and not self.future.done():
                self.future.set_result(status.level)


@pytest.fixture(scope='function')
def yaml_file():
    """Generate a configuration whose floating point parameters are written as integers."""
    with tempfile.NamedTemporaryFile(delete=False) as fp:
        fp.write(
            bytes(
                """
diagnostic_aggregator:
    ros__parameters:
        path: 'robot'
        pub_rate: 2
        analyzers:
            agg:
                type: 'diagnostic_aggregator/GenericAnalyzer'
                path: 'agg'
                contains: ['foo']
                timeout: 2
""",
                'utf-8',
            )
        )
        return fp.name


@pytest.fixture(scope='function')
def diagnostic_aggregator_node():
    return launch_ros.actions.Node(
        name='diagnostic_aggregator',
        package='diagnostic_aggregator',
        executable='aggregator_node',
    )


@launch_pytest.fixture(scope='function')
def launch_description(yaml_file, diagnostic_aggregator_node):
    return launch.LaunchDescription(
        [
            launch_ros.actions.SetParametersFromFile(yaml_file),
            diagnostic_aggregator_node,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=launch_description)
def test_integer_parameters(launch_context):
    rclpy.init()
    node = DiagnosticsTestNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin_until_future_complete(future=node.future, timeout_sec=15.0)
        assert node.future.done(), (
            'No aggregated output; the aggregator probably failed to parse integer parameters'
        )
        assert node.future.result() == DiagnosticStatus.OK
    finally:
        rclpy.try_shutdown()
