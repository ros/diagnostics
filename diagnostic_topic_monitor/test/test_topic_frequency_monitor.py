#! /usr/bin/env python3

# Copyright (c) 2024, 2025 Robert Bosch GmbH
#
# See the top-level LICENSE file for licensing terms.

import pprint
import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
import launch
from launch.actions import EmitEvent
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
import launch.event_handlers.on_process_start
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
import launch_ros.events.lifecycle
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from lifecycle_msgs.msg import Transition
import pytest
import rclpy
from rclpy.time import Duration
from rclpy.time import Time


def create_change_state(target, target_state):
    """Activate the lifecycle monitor nodes."""
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
                LogInfo(msg=f'Emitting configure event for {target_action}'),
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
            start_state='configuring',
            goal_state='inactive',
            entities=[
                        LogInfo(msg=f'Emitting activate event for {target_action}'),
                        create_change_state(
                            target_action, Transition.TRANSITION_ACTIVATE
                        )
            ],
        )
    )


ALL_MONITOR_NAME = 'monitor_all_topics_node'
CONFIG_MONITOR_NAME = 'monitor_configured_topics_node'


@pytest.mark.launch_test
def generate_test_description():
    # Node that publishes the topics we want to monitor
    talker_node = Node(
        package='diagnostic_topic_monitor',
        executable='dummy_publishers.py',
        output='log',
        name='talker',
        arguments=['--ros-args', '--log-level', 'talker:=warn'],
    )
    # Un-configured frequency monitor that checks all topics
    monitor_all_node = LifecycleNode(
        package='diagnostic_topic_monitor',
        executable='topic_frequency_monitor',
        name=ALL_MONITOR_NAME,
        output='both',
        namespace='',
        arguments=['--ros-args', '--log-level', 'all_monitor:=INFO'],
    )
    # Frequency monitor with configuration file
    monitor_config_node = LifecycleNode(
        package='diagnostic_topic_monitor',
        executable='topic_frequency_monitor',
        name=CONFIG_MONITOR_NAME,
        output='both',
        namespace='',
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare('diagnostic_topic_monitor'),
                    'test',
                    'config',
                    'topic_frequency_monitor.yaml',
                ]
            ),
        ],
        arguments=['--ros-args', '--log-level', 'monitor_configured_topics_node:=INFO'],
    )

    return launch.LaunchDescription(
        [
            SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
            talker_node,
            monitor_all_node,
            monitor_config_node,
            # Right after the monitor starts, make it take the 'configure' transition.
            create_register_configure(monitor_all_node),
            create_register_configure(monitor_config_node),
            # When the monitor reaches the 'inactive' state, 'activate'.
            create_register_activate(monitor_all_node),
            create_register_activate(monitor_config_node),
            # When the monitor node reaches the 'active' state, we're ready for testing
            RegisterEventHandler(
                launch_ros.event_handlers.OnStateTransition(
                    target_lifecycle_node=monitor_all_node,
                    start_state='activating',
                    goal_state='active',
                    entities=[
                        LogInfo(msg='Monitor reached active state'),
                        launch_testing.actions.ReadyToTest(),
                    ],
                )
            ),
        ]
    )


class TestMonitor(unittest.TestCase):
    TIMEOUT = 30

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_topic_frequency_monitor_node')
        self.log = self.node.get_logger()
        self.sub = self.node.create_subscription(
            DiagnosticArray, '/diagnostics', self.stat_cb, 10
        )
        self.pub_count = self.node.count_publishers('/diagnostics')
        self.log.info(
            f'Number of publishers for /diagnostics: {self.pub_count}. Listening for messages...'
        )
        self.messages = []
        self.freq_messages = []

        start_time = time.time()
        while len(self.messages) < 5 or len(self.freq_messages) < 5:
            rclpy.spin_once(self.node, timeout_sec=1.0)
            self.log.debug(f'Got {len(self.messages)} and {len(self.freq_messages)}')
            if (time.time() - start_time) > self.TIMEOUT:
                self.fail('Timed out waiting for message in /diagnostics topic')
        self.log.debug(f'Got {len(self.messages)} and {len(self.freq_messages)}')

    def tearDown(self):
        self.node.destroy_node()
        del self.node

    def stat_cb(self, msg):
        """Store message for future processing."""
        if len(msg.status) == 0:
            return
        if msg.status[0].message == 'Node starting up':
            return
        if CONFIG_MONITOR_NAME in msg.status[0].name:
            self.freq_messages.append(msg)
        else:
            self.messages.append(msg)

    def test_diag_msg(self):
        """Check that diagnostics messages contain the right content."""
        pprint. pprint(self.messages)
        last_msg = self.messages.pop()
        # header
        current_time = self.node.get_clock().now()
        header_time = Time.from_msg(last_msg.header.stamp)
        self.assertLess(current_time - header_time, Duration(seconds=1.0), f'{last_msg=}')
        last_status = last_msg.status[0]
        # status should be OK
        self.assertEqual(last_status.level, DiagnosticStatus.OK, f'{last_status}')
        keys = [value.key for value in last_status.values]
        self.assertTrue('period' in keys, f'{last_status}')
        names = [status.name for status in last_msg.status]
        # The all_topics monitor should have all topics
        self.assertIn(f'{ALL_MONITOR_NAME}: /dummy_header_topic', names, f'{last_status}')
        self.assertIn(f'{ALL_MONITOR_NAME}: /dummy_string_topic1', names, f'{last_status}')

    def test_frequency_diag_msg(self):
        """Check that the frequency diagnostic works."""
        last_msg = self.freq_messages.pop()
        self.log.debug(f'{last_msg=}')
        self.assertTrue(len(last_msg.status) > 0, f'{last_msg=}')
        status = last_msg.status[0]
        # check some fields for present/content
        self.assertTrue(CONFIG_MONITOR_NAME in status.name, f'{last_msg=}')
        keys = [kv.key for kv in status.values]
        self.assertIn('Actual frequency (Hz)', keys, f'{last_msg=}')

    def test_ignore_unconfigured(self):
        """Check that we ignore the topic we don't monitor."""
        last_msg = self.freq_messages.pop()
        self.assertEqual(len(last_msg.status), 4, f'{last_msg=}')  # We monitor 4 topics
