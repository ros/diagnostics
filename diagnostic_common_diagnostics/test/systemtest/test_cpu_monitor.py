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
import time
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper
from diagnostic_common_diagnostics.cpu_monitor import CpuTask

class TestCPUMonitor(unittest.TestCase):
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
        # time.sleep(0.1)

        warning_percentage2 = 0
        task2 = CpuTask(warning_percentage2)
        stat2 = DiagnosticStatusWrapper()
        task2.run(stat2)
        self.assertEqual(task2.name, 'CPU Information')
        self.assertEqual(stat2.level, DiagnosticStatus.WARN)
        self.assertIn(str('At least one CPU exceeds'), stat2.message)

        # Check for at least 1 CPU Load Average and 1 CPU Load
        self.assertGreaterEqual(len(stat2.values), 2)
        # time.sleep(0.25)

    # def test_updater(self):


if __name__ == "__main__":
    unittest.main()
