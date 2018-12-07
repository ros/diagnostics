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
# -*- coding: utf-8 -*-

"""@author Brice Rebsamen <brice [dot] rebsamen [gmail]>."""

import rclpy
import diagnostic_updater
from diagnostic_updater import *
import unittest


class TestDiagnosticStatusWrapper(unittest.TestCase):
    def test_init_empty(self):
        d = DiagnosticStatusWrapper()
        self.assertEqual(d.level,b'\x00')
        self.assertEqual(d.message, "")
        self.assertEqual(d.values, [])

    def test_init_lvl_msg(self):
        d = DiagnosticStatusWrapper(level=b'1', message="test")
        self.assertEqual(d.level, b'1')
        self.assertEqual(d.message, "test")
        self.assertEqual(d.values, [])

    def test_summary_lvl_msg(self):
        d = DiagnosticStatusWrapper()
        d.summary(b'1', "test")
        self.assertEqual(d.level, b'1')
        self.assertEqual(d.message, "test")

    def test_summary_dmsg(self):
        d = DiagnosticStatusWrapper(level=b'0', message="ok")
        m = DiagnosticStatus(level=b'1', message="warn")
        d.summary(m)
        self.assertEqual(d.level, b'1')
        self.assertEqual(d.message, "warn")

    def test_clear_summary(self):
        d = DiagnosticStatusWrapper(level=b'0', message="ok")
        d.clearSummary()
        self.assertEqual(d.level, b'0')
        self.assertEqual(d.message, "")

    def test_merge_summary_lvl_msg(self):
        d = DiagnosticStatusWrapper(level=b'0', message="ok")
        d.mergeSummary(b'1', "warn")
        self.assertEqual(d.level, b'1')
        self.assertEqual(d.message, "warn")

        d.mergeSummary(b'2', "err")
        self.assertEqual(d.level, b'2')
        self.assertEqual(d.message, "warn; err")

    def test_merge_summary_dmsg(self):
        d = DiagnosticStatusWrapper(level=b'0', message="ok")
        m = DiagnosticStatus(level=b'1', message="warn")
        d.mergeSummary(m)
        self.assertEqual(d.level, b'1')
        self.assertEqual(d.message, "warn")

        m = DiagnosticStatus(level=b'2', message="err")
        d.mergeSummary(m)
        self.assertEqual(d.level, b'2')
        self.assertEqual(d.message, "warn; err")

    def test_add(self):
        d = DiagnosticStatusWrapper()
        d.add('key','val')
        self.assertEqual(d.values[0].key, 'key')
        self.assertEqual(d.values[0].value, 'val')

if __name__ == '__main__':
    unittest.main()
