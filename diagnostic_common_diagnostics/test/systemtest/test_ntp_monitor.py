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
import subprocess
import unittest

import ament_index_python

from diagnostic_msgs.msg import DiagnosticArray

import rclpy

TIMEOUT_MAX_S = 5.


class TestNTPMonitor(unittest.TestCase):

    def __init__(self, methodName: str = 'runTest') -> None:
        super().__init__(methodName)
        rclpy.init()
        self.n_msgs_received = 0

    def setUp(self):
        self.n_msgs_received = 0
        n = self._count_msgs(TIMEOUT_MAX_S)
        self.assertEqual(n, 0)
        self.subprocess = subprocess.Popen(
            [
                os.path.join(
                    ament_index_python.get_package_prefix(
                        'diagnostic_common_diagnostics'
                    ),
                    'lib',
                    'diagnostic_common_diagnostics',
                    'ntp_monitor.py'
                )
            ]
        )

    def tearDown(self):
        self.subprocess.kill()

    def _diagnostics_callback(self, msg):
        search_strings = [
            'NTP offset from',
            'NTP self-offset for'
        ]
        for search_string in search_strings:
            if search_string not in ''.join([
                s.name for s in msg.status
            ]):
                return
        self.n_msgs_received += 1

    def _count_msgs(self, timeout_s):
        self.n_msgs_received = 0
        node = rclpy.create_node('test_ntp_monitor')
        node.create_subscription(
            DiagnosticArray,
            'diagnostics',
            self._diagnostics_callback,
            1
        )
        TIME_D_S = .05
        waited_s = 0.
        while waited_s < timeout_s and self.n_msgs_received == 0:
            rclpy.spin_once(node, timeout_sec=TIME_D_S)
            waited_s += TIME_D_S
        node.destroy_node()
        return self.n_msgs_received

    def test_publishing(self):
        self.assertEqual(
            self.subprocess.poll(),
            None,
            'NTP monitor subprocess died'
        )

        n = self._count_msgs(TIMEOUT_MAX_S)

        self.assertGreater(
            n,
            0,
            f'No messages received within {TIMEOUT_MAX_S}s'
        )


if __name__ == '__main__':
    unittest.main()
