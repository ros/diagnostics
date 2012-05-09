#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@author Brice Rebsamen <brice [dot] rebsamen [gmail]>
"""

import roslib
roslib.load_manifest('diagnostic_updater')
import rospy
from diagnostic_updater import *
import unittest


class TestDiagnosticStatusWrapper(unittest.TestCase):
    def test_init_empty(self):
        d = DiagnosticStatusWrapper()
        self.assertEqual(d.level, 0)
        self.assertEqual(d.message, "")
        self.assertEqual(d.values, [])

    def test_init_lvl_msg(self):
        d = DiagnosticStatusWrapper(level=1, message="test")
        self.assertEqual(d.level, 1)
        self.assertEqual(d.message, "test")
        self.assertEqual(d.values, [])

    def test_summary_lvl_msg(self):
        d = DiagnosticStatusWrapper()
        d.summary(1, "test")
        self.assertEqual(d.level, 1)
        self.assertEqual(d.message, "test")

    def test_summary_dmsg(self):
        d = DiagnosticStatusWrapper(level=0, message="ok")
        m = DiagnosticStatus(level=1, message="warn")
        d.summary(m)
        self.assertEqual(d.level, 1)
        self.assertEqual(d.message, "warn")

    def test_clear_summary(self):
        d = DiagnosticStatusWrapper(level=0, message="ok")
        d.clearSummary()
        self.assertEqual(d.level, 0)
        self.assertEqual(d.message, "")

    def test_merge_summary_lvl_msg(self):
        d = DiagnosticStatusWrapper(level=0, message="ok")
        d.mergeSummary(1, "warn")
        self.assertEqual(d.level, 1)
        self.assertEqual(d.message, "warn")

        d.mergeSummary(2, "err")
        self.assertEqual(d.level, 2)
        self.assertEqual(d.message, "warn; err")

    def test_merge_summary_dmsg(self):
        d = DiagnosticStatusWrapper(level=0, message="ok")
        m = DiagnosticStatus(level=1, message="warn")
        d.mergeSummary(m)
        self.assertEqual(d.level, 1)
        self.assertEqual(d.message, "warn")

        m = DiagnosticStatus(level=2, message="err")
        d.mergeSummary(m)
        self.assertEqual(d.level, 2)
        self.assertEqual(d.message, "warn; err")

    def test_add(self):
        d = DiagnosticStatusWrapper()
        d.add('key','val')
        self.assertEqual(d.values[0].key, 'key')
        self.assertEqual(d.values[0].value, 'val')

if __name__ == '__main__':
    unittest.main()