# Software License Agreement (BSD License)
#
# Copyright (c) 2026, Robert Bosch GmbH
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
#  * Neither the name of the Willow Garage nor the names of its
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

from collections import namedtuple
import tempfile

# ROS2 includes.
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import launch
import launch_pytest
import launch_ros
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future


# All tests take a common structure.
TestMetadata = namedtuple(
    'TestMetadata',
    ['publish_values', 'diag_msg', 'agg_expected'],
)

# A status value of 'None' means that the state is never sent (it's missing).
TEST_METADATA = [
    # CASE 1: publish_values = True, so we expect the values to exist in the aggregated message
    TestMetadata(
        publish_values=True,
        diag_msg=DiagnosticStatus(
            name='foo',
            level=DiagnosticStatus.WARN,
            message='Foo',
            hardware_id='hw_foo',
            values=[KeyValue(key='foo', value='1')],
        ),
        agg_expected=DiagnosticStatus(
            name='/Agg/foo',
            level=DiagnosticStatus.WARN,
            message='Foo',
            hardware_id='hw_foo',
            values=[KeyValue(key='foo', value='1')],
        ),
    ),
    TestMetadata(
        publish_values=True,
        diag_msg=DiagnosticStatus(
            name='foo',
            level=DiagnosticStatus.WARN,
            message='Foo',
            hardware_id='hw_foo',
            values=[],
        ),
        agg_expected=DiagnosticStatus(
            name='/Agg/foo',
            level=DiagnosticStatus.WARN,
            message='Foo',
            hardware_id='hw_foo',
            values=[],
        ),
    ),
    # CASE 2: publish_values = False, so we expect the values to be empty in the aggregated message
    TestMetadata(
        publish_values=False,
        diag_msg=DiagnosticStatus(
            name='foo',
            level=DiagnosticStatus.WARN,
            message='Foo',
            hardware_id='hw_foo',
            values=[KeyValue(key='foo', value='1')],
        ),
        agg_expected=DiagnosticStatus(
            name='/Agg/foo',
            level=DiagnosticStatus.WARN,
            message='Foo',
            hardware_id='hw_foo',
            values=[],
        ),
    ),
    TestMetadata(
        publish_values=False,
        diag_msg=DiagnosticStatus(
            name='foo',
            level=DiagnosticStatus.WARN,
            message='Foo',
            hardware_id='hw_foo',
            values=[],
        ),
        agg_expected=DiagnosticStatus(
            name='/Agg/foo',
            level=DiagnosticStatus.WARN,
            message='Foo',
            hardware_id='hw_foo',
            values=[],
        ),
    ),
]


class DiagnosticsTestNode(Node):
    """Class that publishes raw diagnostics and listens for aggregated diagnostics."""

    def __init__(self, publish_values, diag_msg, agg_expected):
        super().__init__(node_name='diagnostics_listener_node')
        self.publish_values = publish_values
        self.diag_msg = diag_msg
        self.agg_expected = agg_expected
        self.agg_received = None
        self.counter = 0
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
        """Call from a timer to send off raw diagnostics."""
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'robot'
        if self.diag_msg is not None:
            msg.status.append(self.diag_msg)
        self.publisher.publish(msg)

    def diagnostics_aggregated_callback(self, msg):
        """Call from a subscriber providing aggregated diagnostics."""
        for status in msg.status:
            print(f'Received a status: {status}')
            if status.name == '/Agg/foo':
                self.agg_received = status
                self.counter += 1
                self.future.set_result(self.counter)


@pytest.fixture(scope='function')
def test_metadata(request):
    """Enable parameter indirection, so we can pass a parameterization into fixtures."""
    return request.param


@pytest.fixture(scope='function')
def yaml_file(test_metadata):
    """Generate a YAML file to test a specific configuration state."""
    with tempfile.NamedTemporaryFile(delete=False) as fp:
        fp.write(
            bytes(
                f"""
diagnostic_aggregator:
    ros__parameters:
        publish_values: {test_metadata.publish_values}
        robot:
            type: 'diagnostic_aggregator/GenericAnalyzer'
            path: Agg
            startswith: [ 'foo' ]
""",
                'utf-8',
            )
        )
        return fp.name


@pytest.fixture(scope='function')
def diagnostic_aggregator_node():
    """Declare an aggregator that uses a global configuration set by the launch."""
    return launch_ros.actions.Node(
        name='diagnostic_aggregator',
        package='diagnostic_aggregator',
        executable='aggregator_node',
    )


@launch_pytest.fixture(scope='function')
def launch_description(yaml_file, diagnostic_aggregator_node):
    """Declare what should be launched in each test."""
    return launch.LaunchDescription(
        [
            launch_ros.actions.SetParametersFromFile(yaml_file),
            diagnostic_aggregator_node,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.parametrize('test_metadata', TEST_METADATA, indirect=True)
@pytest.mark.launch(fixture=launch_description)
def test_publish_values(test_metadata, launch_context):
    """Run a launch test for each test in our set of tests."""
    rclpy.init()

    node = DiagnosticsTestNode(
        publish_values=test_metadata.publish_values,
        diag_msg=test_metadata.diag_msg,
        agg_expected=test_metadata.agg_expected,
    )

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin_until_future_complete(future=node.future, timeout_sec=5.0)
        print(
            f"""
        The test produced the following result:
            + Publish values: {test_metadata.publish_values}
            + Published diagnostic message: {node.diag_msg}
            + Expected aggregated message: {test_metadata.agg_expected}
            ----
            + Received aggregated message: {node.agg_received}
        """
        )
        assert node.future.done(), 'Launch timed out without producing aggregation'
        assert (
            node.agg_received == test_metadata.agg_expected
        ), 'Unexpected aggregated message received'
        print(f'It took {node.future.result()} aggregations to find the correct status')

    finally:
        rclpy.try_shutdown()
