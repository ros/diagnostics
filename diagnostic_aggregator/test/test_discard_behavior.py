# Copyright 2023 Open Source Robotics Foundation, Inc.
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
#
# DESCRIPTION
# This test ensures that a parent AnalyzerGroup does not roll up an ERROR state when a
# GenericAnalyzer child block is marked with discard_stale: true and either of these
# conditions are met:
#
#     1. There are no statuses that match any of the GenericAnalyzer child blocks.
#     2. Every matching status in the GenericAnalyzer child block has been marked stale
#
# In this example, if foo and bar have no matching statuses or all of their statuses
# are STALE, they will roll up as OK because the discard_stale: true flag implies that
# stale statuses are disposable.
#
#     analyzer:
#         ros__parameters:
#             path: 'agg'
#             pub_rate: 1.0
#             analyzers:
#                 part:
#                     type: 'diagnostic_aggregator/AnalyzerGroup'
#                     path: 'part'
#                     foo:
#                         type: 'diagnostic_aggregator/GenericAnalyzer'
#                         path: 'foo'
#                         find_and_remove_prefix: ['/foo:']
#                         num_items: 1
#                     bar:
#                         type: 'diagnostic_aggregator/GenericAnalyzer'
#                         path: 'bar'
#                         find_and_remove_prefix: ['/bar:']
#                         discard_stale: true

# Python includes.
from collections import namedtuple
import tempfile

# ROS2 includes.
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
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
    ['foo_discard', 'foo_status', 'bar_discard', 'bar_status', 'agg_expected'],
)

# A status value of 'None' means that the state is never sent (it's missing).
TEST_METADATA = [
    # CASE 1: both 'foo' and 'bar' are marked discard_stale := true
    TestMetadata(
        foo_discard=True,
        foo_status=None,
        bar_discard=True,
        bar_status=None,
        agg_expected=DiagnosticStatus.OK,
    ),
    TestMetadata(
        foo_discard=True,
        foo_status=DiagnosticStatus.STALE,
        bar_discard=True,
        bar_status=DiagnosticStatus.STALE,
        agg_expected=DiagnosticStatus.OK,
    ),
    TestMetadata(
        foo_discard=True,
        foo_status=None,
        bar_discard=True,
        bar_status=DiagnosticStatus.STALE,
        agg_expected=DiagnosticStatus.OK,
    ),
    # CASE 2: both 'foo' and 'bar' are marked discard_stale := false
    TestMetadata(
        foo_discard=False,
        foo_status=None,
        bar_discard=False,
        bar_status=None,
        agg_expected=DiagnosticStatus.STALE,
    ),
    TestMetadata(
        foo_discard=False,
        foo_status=DiagnosticStatus.STALE,
        bar_discard=False,
        bar_status=DiagnosticStatus.STALE,
        agg_expected=DiagnosticStatus.STALE,
    ),
    TestMetadata(
        foo_discard=False,
        foo_status=None,
        bar_discard=False,
        bar_status=DiagnosticStatus.STALE,
        agg_expected=DiagnosticStatus.STALE,
    ),
    # CASE 3: one of 'foo' or 'bar' are marked discard_stale := true
    TestMetadata(
        foo_discard=True,
        foo_status=None,
        bar_discard=False,
        bar_status=None,
        agg_expected=DiagnosticStatus.ERROR,  # <-- This is the case we are testing for.
        # if one of the children is *not* marked discard_stale := true and
        # there are no statuses, then the parent should roll up to ERROR.
    ),
    TestMetadata(
        foo_discard=True,
        foo_status=DiagnosticStatus.OK,
        bar_discard=False,
        bar_status=None,
        agg_expected=DiagnosticStatus.ERROR,
    ),
    TestMetadata(
        foo_discard=True,
        foo_status=None,
        bar_discard=False,
        bar_status=DiagnosticStatus.OK,
        agg_expected=DiagnosticStatus.OK,  # <-- This is the case we are testing for.
        # but if a child is marked discard_stale := true and there are no statuses,
        # the parent should roll up to OK.
    ),
    TestMetadata(
        foo_discard=True,
        foo_status=DiagnosticStatus.OK,
        bar_discard=False,
        bar_status=DiagnosticStatus.OK,
        agg_expected=DiagnosticStatus.OK,
    ),
]


class DiagnosticsTestNode(Node):
    """Class that publishes raw diagnostics and listens for aggregated diagnostics."""

    def __init__(self, foo_status, bar_status, agg_expected):
        super().__init__(node_name='diagnostics_listener_node')
        self.foo_status = foo_status
        self.bar_status = bar_status
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
        if self.foo_status is not None:
            msg.status.append(
                DiagnosticStatus(
                    name='/foo', level=self.foo_status, message='Foo', hardware_id='foo'
                )
            )
        if self.bar_status is not None:
            msg.status.append(
                DiagnosticStatus(
                    name='/bar', level=self.bar_status, message='Bar', hardware_id='bar'
                )
            )
        self.publisher.publish(msg)

    def diagnostics_aggregated_callback(self, msg):
        """Call from a subscriber providing aggregated diagnostics."""
        for status in msg.status:
            if status.name == '/robot/agg':
                self.agg_received = status.level
                self.counter += 1
                if self.agg_expected == status.level:
                    # Diagnostics may take a few iterations to 'settle' into the right
                    # state because of how the STALE logic is applied. So, we keep checking
                    # the aggregator result until it reaches the value we are expecting,
                    # and then trigger the future.
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
        path: 'robot'
        pub_rate: 1.0
        analyzers:
            part:
                type: 'diagnostic_aggregator/AnalyzerGroup'
                path: 'agg'
                timeout: 2.0
                foo:
                    type: 'diagnostic_aggregator/GenericAnalyzer'
                    path: 'foo'
                    find_and_remove_prefix: ['/foo']
                    discard_stale: {test_metadata.foo_discard}
                bar:
                    type: 'diagnostic_aggregator/GenericAnalyzer'
                    path: 'bar'
                    find_and_remove_prefix: ['/bar']
                    discard_stale: {test_metadata.bar_discard}
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
def test_discard_behavior(test_metadata, launch_context):
    """Run a launch test for each test in our set of tests."""
    rclpy.init()

    node = DiagnosticsTestNode(
        foo_status=test_metadata.foo_status,
        bar_status=test_metadata.bar_status,
        agg_expected=test_metadata.agg_expected,
    )

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin_until_future_complete(future=node.future, timeout_sec=10.0)
        print(
            f"""
        The test produced the following result:
            + foo_level: {test_metadata.foo_status} (discard: {test_metadata.foo_discard})
            + bar_level: {test_metadata.bar_status} (discard: {test_metadata.bar_discard})
        Expected level: {test_metadata.agg_expected}
        Received level: {node.agg_received}
        """
        )
        assert node.future.done(), 'Launch timed out without producing aggregation'
        assert (
            node.agg_received == test_metadata.agg_expected
        ), 'Unexpected parent status level'
        print(f'It took {node.future.result()} aggregations to find the correct status')

    finally:
        rclpy.try_shutdown()
