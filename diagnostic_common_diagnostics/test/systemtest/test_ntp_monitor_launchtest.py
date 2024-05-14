#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)
#
# Copyright (c) 2023, Robert Bosch GmbH
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

import unittest

from diagnostic_msgs.msg import DiagnosticArray
import launch
import launch_ros
import launch_testing
from launch_testing_ros import WaitForTopics
import pytest
import rclpy


@pytest.mark.launch_test
def generate_test_description():
    """Launch the ntp_monitor node and return a launch description."""
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='diagnostic_common_diagnostics',
            executable='ntp_monitor.py',
            name='ntp_monitor',
            output='screen',
            arguments=['--offset-tolerance', '100000',
                       '--error-offset-tolerance', '200000',
                       '--ntp_hostname', 'ntp.ubuntu.com']
            # 100s, 200s, we are not testing if your clock is correct
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestNtpMonitor(unittest.TestCase):
    """Test if the ntp_monitor node is publishing diagnostics."""

    def __init__(self, methodName: str = 'runTest') -> None:
        super().__init__(methodName)
        self.received_messages = []

    def _received_message(self, msg):
        self.received_messages.append(msg)

    def _get_min_level(self):
        levels = [
            int.from_bytes(status.level, 'little')
            for diag in self.received_messages
            for status in diag.status]
        if len(levels) == 0:
            return -1
        return min(levels)

    def test_topic_published(self):
        """Test if the ntp_monitor node is publishing diagnostics."""
        with WaitForTopics(
            [('/diagnostics', DiagnosticArray)],
            timeout=5
        ):
            print('Topic found')

        rclpy.init()
        test_node = rclpy.create_node('test_node')
        test_node.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self._received_message,
            1
        )

        while len(self.received_messages) < 10:
            rclpy.spin_once(test_node, timeout_sec=1)
            if (min_level := self._get_min_level()) == 0:
                break

        test_node.destroy_node()
        rclpy.shutdown()
        print(f'Got {len(self.received_messages)} messages:')
        for msg in self.received_messages:
            print(msg)
        self.assertEqual(min_level, 0)
