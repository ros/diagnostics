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

import time
import unittest

from diagnostic_common_diagnostics.cpu_monitor import CpuTask

from diagnostic_msgs.msg import DiagnosticStatus

from diagnostic_updater import DiagnosticArray, Updater
from diagnostic_updater import DiagnosticStatusWrapper

import rclpy
from rclpy.node import Node


class TestCPUMonitor(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=None)

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        # In this case is recommended for accuracy that psutil.cpu_percent()
        # function be called with at least 0.1 seconds between calls.
        time.sleep(0.1)

    def diagnostics_callback(self, msg):
        self.message_recieved = True
        self.assertEqual(len(msg.status), 1)

    def test_ok(self):
        warning_percentage = 100
        task = CpuTask(warning_percentage)
        stat = DiagnosticStatusWrapper()
        task.run(stat)
        self.assertEqual(task.name, 'CPU Information')
        self.assertEqual(stat.level, DiagnosticStatus.OK)
        self.assertIn(str('CPU Average'), stat.message)

        # Check for at least 1 CPU Load Average and 1 CPU Load
        self.assertGreaterEqual(len(stat.values), 2)

    def test_warn(self):
        warning_percentage = -1
        task = CpuTask(warning_percentage)
        stat = DiagnosticStatusWrapper()
        task.run(stat)
        print(f'Raw readings: {task._readings}')
        self.assertEqual(task.name, 'CPU Information')
        self.assertEqual(stat.level, DiagnosticStatus.WARN)
        self.assertIn(str('At least one CPU exceeds'), stat.message)

        # Check for at least 1 CPU Load Average and 1 CPU Load
        self.assertGreaterEqual(len(stat.values), 2)

    def test_updater(self):
        self.message_recieved = False

        node = Node('cpu_monitor_test')
        updater = Updater(node)
        updater.setHardwareID('test_id')
        updater.add(CpuTask())

        node.create_subscription(
            DiagnosticArray, '/diagnostics', self.diagnostics_callback, 10)

        start_time = time.time()
        timeout = 5.0  # Timeout in seconds

        while not self.message_recieved:
            rclpy.spin_once(node)
            time.sleep(0.1)
            elapsed_time = time.time() - start_time
            if elapsed_time >= timeout:
                self.fail('No diagnostics received')


if __name__ == '__main__':
    unittest.main()
