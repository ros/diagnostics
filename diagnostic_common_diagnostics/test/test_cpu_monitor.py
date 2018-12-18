#!/usr/bin/env python
# Copyright 2015 Open Source Robotics Foundation, Inc.
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

# \author Rein Appeldoorn

import os
import pathlib
import unittest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from unittest.mock import Mock
from time import sleep
from rclpy.task import Future
from rclpy.task import Task

from launch import LaunchDescription
from launch import LaunchService
from launch import LaunchIntrospector
from launch_ros import get_default_launch_description
from launch.substitutions import EnvironmentVariable
import launch_ros.actions.node
import yaml
import threading

TEST_NODE = 'my_node_cpu'
TEST_NAMESPACE = '/'

class TestCPUMonitor(unittest.TestCase):
        
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node(TEST_NODE, namespace=TEST_NAMESPACE)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def diagnostics_callback(self, msg):
        self._msg = msg
        print("diagnostics_callback called",msg)
        #self.node._subscriber.unregister()
        self.node.destroy_node()
        self.ls.shutdown()

        self.assertEqual(len(self._msg.status), 1)
        status = self._msg.status[0]

        for v in status.values:
            percentage = float(v.value)
            self.assertGreaterEqual(percentage, 0)
            self.assertLessEqual(percentage, 100)

        if self._expected_level:
            self.assertEqual(self._expected_level, status.level)

        self.Test_pass = True
    

    def _assert_launch_errors(self, actions):
        ld = LaunchDescription(actions)
        self.ls = LaunchService()
        self.ls.include_launch_description(ld)
        assert 0 != self.ls.run()

    def _assert_launch_no_errors(self, actions):
        ld = LaunchDescription(actions)
        self.ls = LaunchService()
        self.ls.include_launch_description(ld)
        t = threading.Thread(target=self.ls.run, kwargs={'shutdown_when_idle': False})
        t.start()
        #assert 0 == self.ls.run()

    def test_cpu_monitor_diagnostics(self):
       # rospy.init_node('test_cpu_monitor')
        #rclpy.init()
        #self.node = rclpy.create_node( TEST_NODE)
        self.Test_pass = False
        self._expected_level = None
        self._msg = None
        self._subscriber = self.node.create_subscription(DiagnosticArray,'diagnostics', self.diagnostics_callback)
        self._expected_level = b'\x00'
        node_action = launch_ros.actions.Node(
            package='diagnostic_common_diagnostics', node_executable='cpu_monitor', output='screen')
        self._assert_launch_no_errors([node_action])
        print("will go for spining once")
        while self.Test_pass ==False:
            rclpy.spin_once(self.node)
        #self._subscriber = rospy.Subscriber('diagnostics', DiagnosticArray, self.diagnostics_callback)


'''
    def test_diagnostics_callback(self):
        ld = LaunchDescription([
        launch_ros.actions.Node(
            package='demo_nodes_cpp', node_executable='talker', output='screen',
            remappings=[('chatter', 'my_chatter')]),
        launch_ros.actions.Node(
            package='diagnostic_common_diagnostics', node_executable='cpu_monitor', output='screen',
            ),
        ])

        print('Starting introspection of launch description...')
        print('')

        print(LaunchIntrospector().format_launch_description(ld))

        print('')
        print('Starting launch of launch description...')
        print('')

        # ls = LaunchService(debug=True)
        ls = LaunchService()
        ls.include_launch_description(get_default_launch_description(prefix_output_with_name=False))
        ls.include_launch_description(ld)
        print("After launch service ")
        assert 0 == ls.run()
'''
PKG = 'diagnostics_common_diagnostics'
NAME = 'test_cpu_monitor'
if __name__ == '__main__':
    unittest.main()
