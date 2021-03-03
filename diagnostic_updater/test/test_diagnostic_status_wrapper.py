#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""@author Brice Rebsamen <brice [dot] rebsamen [gmail]>."""

import unittest

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper


class TestDiagnosticStatusWrapper(unittest.TestCase):

    def test_init_empty(self):
        d = DiagnosticStatusWrapper()
        self.assertEqual(d.level, b'\x00')
        self.assertEqual(d.message, '')
        self.assertEqual(d.values, [])

    def test_init_lvl_msg(self):
        d = DiagnosticStatusWrapper(level=b'\x01', message='test')
        self.assertEqual(d.level, b'\x01')
        self.assertEqual(d.message, 'test')
        self.assertEqual(d.values, [])

    def test_summary_lvl_msg(self):
        d = DiagnosticStatusWrapper()
        d.summary(b'\x01', 'test')
        self.assertEqual(d.level, b'\x01')
        self.assertEqual(d.message, 'test')

    def test_summary_dmsg(self):
        d = DiagnosticStatusWrapper(level=b'\x00', message='ok')
        m = DiagnosticStatus(level=b'\x01', message='warn')
        d.summary(m)
        self.assertEqual(d.level, b'\x01')
        self.assertEqual(d.message, 'warn')

    def test_clear_summary(self):
        d = DiagnosticStatusWrapper(level=b'\x00', message='ok')
        d.clearSummary()
        self.assertEqual(d.level, b'\x00')
        self.assertEqual(d.message, '')

    def test_merge_summary_lvl_msg(self):
        d = DiagnosticStatusWrapper(level=b'\x00', message='ok')
        d.mergeSummary(b'\x01', 'warn')
        self.assertEqual(d.level, b'\x01')
        self.assertEqual(d.message, 'warn')

        d.mergeSummary(b'\x02', 'err')
        self.assertEqual(d.level, b'\x02')
        self.assertEqual(d.message, 'warn; err')

    def test_merge_summary_dmsg(self):
        d = DiagnosticStatusWrapper(level=b'\x00', message='ok')
        m = DiagnosticStatus(level=b'\x01', message='warn')
        d.mergeSummary(m)
        self.assertEqual(d.level, b'\x01')
        self.assertEqual(d.message, 'warn')

        m = DiagnosticStatus(level=b'\x02', message='err')
        d.mergeSummary(m)
        self.assertEqual(d.level, b'\x02')
        self.assertEqual(d.message, 'warn; err')

    def test_add(self):
        d = DiagnosticStatusWrapper()
        d.add('key', 'val')
        self.assertEqual(d.values[0].key, 'key')
        self.assertEqual(d.values[0].value, 'val')


if __name__ == '__main__':
    unittest.main()
