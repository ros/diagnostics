#! /usr/bin/env python3

# Copyright (c) 2024, 2025 Robert Bosch GmbH
#
# See the top-level LICENSE file for licensing terms.

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
    # Monitor with configuration file
    monitor_config_node = LifecycleNode(
        package='diagnostic_topic_monitor',
        executable='topic_age_monitor',
        name=CONFIG_MONITOR_NAME,
        output='both',
        namespace='',
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare('diagnostic_topic_monitor'),
                    'test',
                    'config',
                    'topic_age_monitor.yaml',
                ]
            ),
        ],
        arguments=['--ros-args', '--log-level', 'monitor_configured_topics_node:=INFO'],
    )

    return launch.LaunchDescription(
        [
            SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
            talker_node,
            monitor_config_node,
            # Right after the monitor starts, make it take the 'configure' transition.
            create_register_configure(monitor_config_node),
            # When the monitor reaches the 'inactive' state, 'activate'.
            create_register_activate(monitor_config_node),
            # When the monitor node reaches the 'active' state, we're ready for testing
            RegisterEventHandler(
                launch_ros.event_handlers.OnStateTransition(
                    target_lifecycle_node=monitor_config_node,
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
        self.node = rclpy.create_node('test_topic_age_monitor_node')
        self.log = self.node.get_logger()
        self.sub = self.node.create_subscription(
            DiagnosticArray, '/diagnostics', self.stat_cb, 10
        )
        self.pub_count = self.node.count_publishers('/diagnostics')
        self.log.info(
            f'Number of publishers for /diagnostics: {self.pub_count}. Listening for messages...'
        )
        self.age_messages = []  # Store messages from configured age monitor

        start_time = time.time()
        while len(self.age_messages) < 3:
            rclpy.spin_once(self.node, timeout_sec=1.0)
            self.log.debug(f'Got {len(self.age_messages)} age msgs msgs')
            if (time.time() - start_time) > self.TIMEOUT:
                self.fail('Timed out waiting for message in /diagnostics topic')
        self.log.info(f'Stored {len(self.age_messages)} age msgs')

    def tearDown(self):
        self.node.destroy_node()
        del self.node

    def stat_cb(self, msg):
        """Store message for future processing."""
        if len(msg.status) == 0:
            return
        if CONFIG_MONITOR_NAME in msg.status[0].name:
            self.age_messages.append(msg)

    def test_diag_msg(self):
        """Check that diagnostics messages contain the right content."""
        last_msg = self.age_messages.pop()
        # header
        current_time = self.node.get_clock().now()
        header_time = Time.from_msg(last_msg.header.stamp)
        self.assertLess(current_time - header_time, Duration(seconds=0.1))
        last_status = last_msg.status[0]
        # status should be OK
        self.assertEqual(last_status.level, DiagnosticStatus.OK)

    def test_age_diag_msg(self):
        """Check that the age diagnostic works."""
        last_msg = self.age_messages.pop()
        self.assertTrue(len(last_msg.status) > 0)
        status = last_msg.status[0]
        # check some fields for present/content
        self.assertTrue(CONFIG_MONITOR_NAME in status.name)
        keys = [kv.key for kv in status.values]
        self.assertTrue('Earliest timestamp delay:' in keys)

    def test_ignore_unconfigured(self):
        """Check that we ignore the topic we don't monitor."""
        last_msg = self.age_messages.pop()
        self.assertEqual(len(last_msg.status), 1)  # We monitor 1 topic only
        names = [status.name for status in last_msg.status]
        #  This topic should be monitored
        self.assertIn(f'{CONFIG_MONITOR_NAME}: /dummy_header_topic', names, f'{names}')
        # This topic should not be monitored
        self.assertNotIn(
            f'{CONFIG_MONITOR_NAME}: /dummy_string_topic1',
            names,
            f'{names}',
        )
