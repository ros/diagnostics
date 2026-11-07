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
Test the "persistent_state" handling of the aggregator.

A status that carries the reserved key-value "persistent_state: True" must keep
its last level and message after its publisher stops updating it, instead of
being turned STALE (GenericAnalyzer) or discarded (discard_stale, OtherAnalyzer).
Publishing the status again without the key restores the normal timeout
behaviour, and the analyzer's persistent_timeout caps persistence.
"""

from collections import namedtuple
import tempfile

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import launch
import launch_pytest
import launch_ros
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future


# Analyzer timeout in the generated configuration, and how long the test waits
# after the last raw message before inspecting the aggregated output. The settle
# time exceeds the 5 s discard timeout of the OtherAnalyzer on purpose.
ANALYZER_TIMEOUT = 1.0
SETTLE_TIME = 6.0

TestMetadata = namedtuple(
    'TestMetadata',
    [
        'persistent_msgs',  # number of raw messages carrying persistent_state: True
        'plain_msgs',  # number of raw messages sent afterwards without the key
        'discard_stale',  # GenericAnalyzer discard_stale parameter
        'persistent_timeout',  # GenericAnalyzer persistent_timeout parameter
        'foo_expected',  # expected level of /robot/agg/foo, None if it must be absent
        'foo_has_age',  # whether /robot/agg/foo must carry last_update_age
        'agg_expected',  # expected level of the /robot/agg header
        'other_expected',  # expected level of /robot/Other/other, None if absent
    ],
)

TEST_METADATA = [
    # Baseline: without the key everything goes stale / gets discarded.
    TestMetadata(
        persistent_msgs=0,
        plain_msgs=5,
        discard_stale=False,
        persistent_timeout=-1.0,
        foo_expected=DiagnosticStatus.STALE,
        foo_has_age=False,
        agg_expected=DiagnosticStatus.STALE,
        other_expected=None,
    ),
    # Persistent items keep their level and message, in GenericAnalyzer and in Other.
    TestMetadata(
        persistent_msgs=5,
        plain_msgs=0,
        discard_stale=False,
        persistent_timeout=-1.0,
        foo_expected=DiagnosticStatus.WARN,
        foo_has_age=True,
        agg_expected=DiagnosticStatus.WARN,
        other_expected=DiagnosticStatus.WARN,
    ),
    # Persistent items are not discarded either.
    TestMetadata(
        persistent_msgs=5,
        plain_msgs=0,
        discard_stale=True,
        persistent_timeout=-1.0,
        foo_expected=DiagnosticStatus.WARN,
        foo_has_age=True,
        agg_expected=DiagnosticStatus.WARN,
        other_expected=DiagnosticStatus.WARN,
    ),
    # persistent_timeout caps persistence; afterwards the normal timeout applies.
    TestMetadata(
        persistent_msgs=5,
        plain_msgs=0,
        discard_stale=False,
        persistent_timeout=2.0,
        foo_expected=DiagnosticStatus.STALE,
        foo_has_age=False,
        agg_expected=DiagnosticStatus.STALE,
        other_expected=DiagnosticStatus.WARN,
    ),
    # Sending the status again without the key restores the timeout behaviour.
    TestMetadata(
        persistent_msgs=3,
        plain_msgs=2,
        discard_stale=False,
        persistent_timeout=-1.0,
        foo_expected=DiagnosticStatus.STALE,
        foo_has_age=False,
        agg_expected=DiagnosticStatus.STALE,
        other_expected=None,
    ),
]


class DiagnosticsTestNode(Node):
    """Publishes a burst of raw diagnostics, then inspects the aggregated output."""

    def __init__(self, metadata):
        super().__init__(node_name='persistent_state_test_node')
        self.metadata = metadata
        self.sent = 0
        self.stop_time = None
        self.result = None
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
        """Send the raw diagnostics burst, then remember when it stopped."""
        total = self.metadata.persistent_msgs + self.metadata.plain_msgs
        if self.sent >= total:
            if self.stop_time is None:
                self.stop_time = self.get_clock().now()
            return

        persistent = self.sent < self.metadata.persistent_msgs
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name in ('foo', 'other'):
            status = DiagnosticStatus(
                name=name, level=DiagnosticStatus.WARN, message='Latched', hardware_id=name
            )
            if persistent:
                status.values.append(KeyValue(key='persistent_state', value='True'))
            msg.status.append(status)
        self.publisher.publish(msg)
        self.sent += 1

    def diagnostics_aggregated_callback(self, msg):
        """Keep the first aggregated message that arrives after the settle time."""
        if self.stop_time is None or self.future.done():
            return
        elapsed = (self.get_clock().now() - self.stop_time).nanoseconds * 1e-9
        if elapsed < SETTLE_TIME:
            return
        self.result = msg
        self.future.set_result(True)


def find_status(msg, name):
    for status in msg.status:
        if status.name == name:
            return status
    return None


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
        pub_rate: 2.0
        analyzers:
            agg:
                type: 'diagnostic_aggregator/GenericAnalyzer'
                path: 'agg'
                contains: ['foo']
                timeout: {ANALYZER_TIMEOUT}
                discard_stale: {test_metadata.discard_stale}
                persistent_timeout: {test_metadata.persistent_timeout}
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
def test_persistent_state(test_metadata, launch_context):
    """Run a launch test for each entry in TEST_METADATA."""
    rclpy.init()

    node = DiagnosticsTestNode(test_metadata)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin_until_future_complete(future=node.future, timeout_sec=30.0)
        assert node.future.done(), 'Launch timed out without producing aggregation'
        msg = node.result
        names = [status.name for status in msg.status]
        print(f'Aggregated statuses after settling: {names}')

        foo = find_status(msg, '/robot/agg/foo')
        if test_metadata.foo_expected is None:
            assert foo is None, 'Expected /robot/agg/foo to be absent'
        else:
            assert foo is not None, 'Expected /robot/agg/foo to be present'
            assert foo.level == test_metadata.foo_expected, 'Unexpected level of /robot/agg/foo'
            assert foo.message == 'Latched', 'Persistent item must keep its message'
            keys = [kv.key for kv in foo.values]
            assert ('last_update_age' in keys) == test_metadata.foo_has_age, (
                f'Unexpected last_update_age presence, values: {keys}'
            )
            if test_metadata.foo_has_age:
                age = float(next(kv.value for kv in foo.values if kv.key == 'last_update_age'))
                assert age >= SETTLE_TIME - 1.0, f'last_update_age {age} too small'

        agg = find_status(msg, '/robot/agg')
        assert agg is not None, 'Expected /robot/agg header to be present'
        assert agg.level == test_metadata.agg_expected, 'Unexpected level of /robot/agg header'

        other = find_status(msg, '/robot/Other/other')
        if test_metadata.other_expected is None:
            assert other is None, 'Expected /robot/Other/other to be discarded'
        else:
            assert other is not None, 'Expected /robot/Other/other to be kept'
            assert other.level == test_metadata.other_expected, (
                'Unexpected level of /robot/Other/other'
            )
            assert other.message == 'Latched', 'Persistent item must keep its message'

    finally:
        rclpy.try_shutdown()
