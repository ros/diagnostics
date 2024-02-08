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

import os

import ament_index_python

import launch

import launch_ros

import launch_pytest
from launch_pytest.tools import process as process_tools

from launch_testing_ros import WaitForTopics

import launch_testing

import pytest

import unittest

import rclpy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


@pytest.mark.launch_test
def generate_test_description():
    # Launch a process to test
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='diagnostic_common_diagnostics',
            executable='ntp_monitor.py',
            name='ntp_monitor',
            output='screen',
            arguments=['--offset-tolerance', '10000', '--error-offset-tolerance', '20000']  
            # 10s, 20s, we are not testing if your clock is correct
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestGoodProcess(unittest.TestCase):
    def __init__(self, methodName: str = "runTest") -> None:
        super().__init__(methodName)
        self.received_messages = []

    def _received_message(self, msg):
        self.received_messages.append(msg)

    def test_topic_published(self):
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

        test_node.destroy_node()

        min_level = 10
        for msg in self.received_messages:
            for status in msg.status:
                level = int.from_bytes(status.level, byteorder='little')
                print('Level: ', level)
                if level < min_level:
                    min_level = level
        
        self.assertEqual(min_level, 0)