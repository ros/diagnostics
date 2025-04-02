#! /usr/bin/env python3

# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

# #
# # Copyright 2019 Open Source Robotics Foundation, Inc.
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.

import time
import unittest
import pytest

import launch
from launch.actions import EmitEvent, RegisterEventHandler, LogInfo, SetEnvironmentVariable
import launch.event_handlers.on_process_start

from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
import launch_ros.events.lifecycle

import launch_testing
import launch_testing.actions
import launch_testing.asserts

from lifecycle_msgs.msg import Transition
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

import rclpy
from rclpy.time import Time, Duration


def create_change_state(target, target_state):
    return EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(target),
            transition_id=target_state,
        )
    )


def create_register_configure(target_action):
    return RegisterEventHandler(
        launch.event_handlers.on_process_start.OnProcessStart(
            target_action=target_action,
            on_start=[
                LogInfo(msg=f"Emitting configure event for {target_action}"),
                create_change_state(
                    target_action, Transition.TRANSITION_CONFIGURE
                )
            ],
        )
    )


def create_register_activate(target_action):
    return RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=target_action,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                        LogInfo(msg=f"Emitting activate event for {target_action}"),
                        create_change_state(
                            target_action, Transition.TRANSITION_ACTIVATE
                        )
            ],
        )
    )


ALL_MONITOR_NAME = "all_monitor"
FREQ_MONITOR_NAME = "frequency_monitor"


@pytest.mark.launch_test
def generate_test_description():
    monitor_node = LifecycleNode(
        package="rob_topic_monitor",
        executable="topic_monitor",
        name=ALL_MONITOR_NAME,
        output="both",
        namespace="",
        arguments=['--ros-args', '--log-level', 'all_monitor:=INFO']
    )
    frequency_monitor_node = LifecycleNode(
        package="rob_topic_monitor",
        executable="topic_monitor",
        name=FREQ_MONITOR_NAME,
        output="both",
        namespace="",
        parameters=[{"topics": ["/topic"],
                     'min_freqs': [1.8],
                     'max_freqs': [2.1],
                     'diag_prefix': 'freq',
                     'monitor_configured_only': True,
                     'diagnostic_updater.use_fqn': True}],
        arguments=['--ros-args', '--log-level', 'frequency_monitor:=INFO']
    )
    talker_node = Node(
        package="examples_rclcpp_minimal_publisher",
        executable="publisher_lambda",
        output="log",
        name="talker",
        arguments=['--ros-args', '--log-level', 'talker:=warn']
    )
    talker2_node = Node(
        package="examples_rclcpp_minimal_publisher",
        executable="publisher_lambda",
        name="talker2",
        output="log",
        remappings=[("topic", "ignore_topic")],
        arguments=['--ros-args', '--log-level', 'talker2:=warn']
    )
    return launch.LaunchDescription(
        [
            SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
            talker_node,
            talker2_node,
            monitor_node,
            frequency_monitor_node,
            # Right after the monitor starts, make it take the 'configure' transition.
            create_register_configure(monitor_node),
            create_register_configure(frequency_monitor_node),
            # When the monitor reaches the 'inactive' state, 'activate'.
            create_register_activate(monitor_node),
            create_register_activate(frequency_monitor_node),
            # When the monitor node reaches the 'active' state, we're ready for testing
            RegisterEventHandler(
                launch_ros.event_handlers.OnStateTransition(
                    target_lifecycle_node=monitor_node,
                    start_state="activating",
                    goal_state="active",
                    entities=[
                        LogInfo(msg="Monitor reached active state"),
                        launch_testing.actions.ReadyToTest(),
                    ],
                )
            ),
        ]
    )


@pytest.mark.skip(reason="Brittle on CI as long as we don't have proper resource constraints")
class TestMonitor(unittest.TestCase):
    TIMEOUT = 30

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_topic_monitor_node")
        self.log = self.node.get_logger()
        self.sub = self.node.create_subscription(
            DiagnosticArray, "/diagnostics", self.stat_cb, 10
        )
        self.pub_count = self.node.count_publishers("/diagnostics")
        self.log.info(f"Number of publishers for /diagnostics: {self.pub_count}")
        self.messages = []
        self.freq_messages = []

        start_time = time.time()
        while len(self.messages) < 3 or len(self.freq_messages) < 3:
            rclpy.spin_once(self.node, timeout_sec=1.0)
            self.log.debug(f"Got {len(self.messages)} and {len(self.freq_messages)}")
            if (time.time() - start_time) > self.TIMEOUT:
                self.fail("Timed out waiting for message in /diagnostics topic")
        self.log.debug(f"Got {len(self.messages)} and {len(self.freq_messages)}")

    def tearDown(self):
        self.node.destroy_node()
        del self.node

    def stat_cb(self, msg):
        """Store message for future processing."""
        if len(msg.status) == 0:
            return
        if FREQ_MONITOR_NAME in msg.status[0].name:
            self.freq_messages.append(msg)
        else:
            self.messages.append(msg)

    def test_diag_msg(self):
        """Check that diagnostics messages contain the right content."""
        last_msg = self.messages.pop()
        # header
        current_time = self.node.get_clock().now()
        header_time = Time.from_msg(last_msg.header.stamp)
        self.assertLess(current_time - header_time, Duration(seconds=0.1))
        last_status = last_msg.status[0]
        # status should be OK
        self.assertEqual(last_status.level, DiagnosticStatus.OK)
        # period estimate should be 4 Hz
        keys = [value.key for value in last_status.values]
        self.assertTrue("period" in keys)
        # disabled until we can get deterministic measurement on CI
        # self.assertAlmostEqual(
        #    float(last_status.values[keys.index("period")].value), 0.25, delta=0.02)
        # check that the two topics of this test are present
        # (a bit more complex, since on CI other topics may be running in parallel)
        names = [status.name for status in last_msg.status]
        self.assertIn(f"{ALL_MONITOR_NAME}: /topic", names, f"{names}")
        self.assertIn(f"{ALL_MONITOR_NAME}: /ignore_topic", names, f"{names}")

    def test_frequency_diag_msg(self):
        """Check that the frequency diagnostic works."""
        last_msg = self.freq_messages.pop()
        self.log.debug(f"{last_msg}")
        self.assertTrue(len(last_msg.status) > 0)
        status = last_msg.status[0]
        # check some fields for present/content
        self.assertTrue(FREQ_MONITOR_NAME in status.name)
        keys = [kv.key for kv in status.values]
        self.assertIn("Actual frequency (Hz)", keys)
        # disabled until we can get deterministic measurement on CI
        # freq = [kv.value for kv in status.values if kv.key == "Actual frequency (Hz)"][0]
        # self.assertAlmostEqual(float(freq), 2.0, delta=0.02)

    def test_ignore_unconfigured(self):
        """Check that we ignore the topic we don't monitor."""
        last_msg = self.freq_messages.pop()
        self.assertEqual(len(last_msg.status), 1)
