#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""@author Brice Rebsamen <brice [dot] rebsamen [gmail]>."""

import time
import unittest

from diagnostic_updater import DiagnosticStatusWrapper, DiagnosticTask
from diagnostic_updater import FrequencyStatus, FrequencyStatusParam
from diagnostic_updater import TimeStampStatus, Updater

import rclpy
from rclpy.clock import Clock
from rclpy.clock import ClockType


class ClassFunction(DiagnosticTask):

    def __init__(self):
        DiagnosticTask.__init__(self, 'classFunction')

    def run(self, stat):
        stat.summary(b'\x00', 'Test is running')
        stat.add('Value', '%f' % 5)
        stat.add('String', 'Toto')
        stat.add('Floating', 5.55)
        stat.add('Integer', 5)
        stat.add('Formatted', '%s %i', 'Hello', 5)
        stat.add('Bool', True)
        return stat


class TestClass:

    def wrapped(stat):
        return stat


class TestDiagnosticStatusWrapper(unittest.TestCase):

    def testDiagnosticUpdater(self):
        rclpy.init()
        node = rclpy.create_node('test_node')
        updater = Updater(node)

        c = TestClass()
        updater.add('wrapped', c.wrapped)

        cf = ClassFunction()
        updater.add(cf)

    def testDiagnosticStatusWrapper(self):
        stat = DiagnosticStatusWrapper()

        message = 'dummy'
        level = b'\x01'
        stat.summary(level, message)
        self.assertEqual(message, stat.message, 'DiagnosticStatusWrapper::summary failed\
                         to set message')
        self.assertEqual(level, stat.level, 'DiagnosticStatusWrapper::summary failed to set level')

        stat.add('toto', '%.1f' % 5.0)
        stat.add('baba', '5')
        stat.add('foo', '%05i' % 27)
        stat.add('bool', 'True')
        stat.add('bool2', 'False')
        self.assertEqual('5.0', stat.values[0].value, 'Bad value, adding a value with addf')
        self.assertEqual('5', stat.values[1].value, 'Bad value, adding a string with add')
        self.assertEqual('00027', stat.values[2].value, 'Bad value, adding a string with addf')
        self.assertEqual('toto', stat.values[0].key, 'Bad label, adding a value with add')
        self.assertEqual('baba', stat.values[1].key, 'Bad label, adding a string with add')
        self.assertEqual('foo', stat.values[2].key, 'Bad label, adding a string with addf')

        self.assertEqual('bool', stat.values[3].key, 'Bad label, adding a true bool key with add')
        self.assertEqual('True', stat.values[3].value, 'Bad value, adding a true bool with add')

        self.assertEqual('bool2', stat.values[4].key, 'Bad label, adding a false bool key\
                         with add')
        self.assertEqual('False', stat.values[4].value, 'Bad value, adding a false bool with add')

    def testFrequencyStatus(self):
        freq_bound = {'min': 10, 'max': 20}

        fs = FrequencyStatus(FrequencyStatusParam(freq_bound, 0.5, 2))

        stat = [DiagnosticStatusWrapper() for i in range(5)]
        fs.tick()
        time.sleep(.02)
        stat[0] = fs.run(stat[0])
        # Should be too fast, 20 ms for 1 tick, lower limit should be 33ms.
        time.sleep(.05)
        fs.tick()
        stat[1] = fs.run(stat[1])
        # Should be good, 70 ms for 2 ticks, lower limit should be 66 ms.
        time.sleep(.3)
        fs.tick()
        stat[2] = fs.run(stat[2])
        # Should be good, 350 ms for 2 ticks, upper limit should be 400 ms.
        time.sleep(.15)
        fs.tick()
        stat[3] = fs.run(stat[3])
        # Should be too slow, 450 ms for 2 ticks, upper limit should be 400 ms.
        fs.clear()
        stat[4] = fs.run(stat[4])
        # Should be good, just cleared it.

        self.assertEqual(b'\x01', stat[0].level, 'max frequency exceeded but not reported')
        self.assertEqual(b'\x00', stat[1].level, 'within max frequency but reported error')
        self.assertEqual(b'\x00', stat[2].level, 'within min frequency but reported error')
        self.assertEqual(b'\x01', stat[3].level, 'min frequency exceeded but not reported')
        self.assertEqual(b'\x02', stat[4].level, 'freshly cleared should fail')
        self.assertEqual('', stat[0].name, 'Name should not be set by FrequencyStatus')
        self.assertEqual('FrequencyStatus', fs.getName(), 'Name should be Frequency Status')

    def testTimeStampStatus(self):
        ts = TimeStampStatus()

        stat = [DiagnosticStatusWrapper() for i in range(5)]
        stat[0] = ts.run(stat[0])
        clock = Clock(clock_type=ClockType.STEADY_TIME)
        now = clock.now()
        ts.tick((now.nanoseconds * 1e-9) + 2)
        stat[1] = ts.run(stat[1])
        now = clock.now()
        ts.tick((now.nanoseconds * 1e-9))
        stat[2] = ts.run(stat[2])
        now = clock.now()
        ts.tick((now.nanoseconds * 1e-9) - 4)
        stat[3] = ts.run(stat[3])
        now = clock.now()
        ts.tick((now.nanoseconds * 1e-9) - 6)
        stat[4] = ts.run(stat[4])

        self.assertEqual(b'\x01', stat[0].level, 'no data should return a warning')
        self.assertEqual(b'\x02', stat[1].level, 'too far future not reported')
        self.assertEqual(b'\x00', stat[2].level, 'now not accepted')
        self.assertEqual(b'\x00', stat[3].level, '4 seconds ago not accepted')
        self.assertEqual(b'\x02', stat[4].level, 'too far past not reported')
        self.assertEqual('', stat[0].name, 'Name should not be set by TimeStapmStatus')
        self.assertEqual('Timestamp Status', ts.getName(), 'Name should be Timestamp Status')


if __name__ == '__main__':
    rclpy.init()
    unittest.main()
