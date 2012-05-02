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
import time

class ClassFunction(DiagnosticTask):
    def __init__(self):
        DiagnosticTask.__init__(self, "classFunction")

    def run(self, stat):
        stat.summary(0, "Test is running")
        stat.add("Value", "%f" % 5)
        stat.add("String", "Toto")
        stat.add("Floating", 5.55)
        stat.add("Integer", 5)
        stat.add("Formatted", "%s %i", "Hello", 5)
        stat.add("Bool", True)
        return stat

class TestClass:
    def wrapped(stat):
        return stat


class TestDiagnosticStatusWrapper(unittest.TestCase):

    def testDiagnosticUpdater(self):
        updater = Updater()

        c = TestClass()
        updater.add("wrapped", c.wrapped)

        cf = ClassFunction()
        updater.add(cf)


    def testDiagnosticStatusWrapper(self):
        stat = DiagnosticStatusWrapper()

        message = "dummy"
        level = 1
        stat.summary(level, message)
        self.assertEqual(message, stat.message, "DiagnosticStatusWrapper::summary failed to set message")
        self.assertEqual(level, stat.level, "DiagnosticStatusWrapper::summary failed to set level")

        stat.add("toto", "%.1f" % 5.0)
        stat.add("baba", 5)
        stat.add("foo", "%05i" % 27)
        stat.add("bool", True)
        stat.add("bool2", False)

        self.assertEqual("5.0", stat.values[0].value, "Bad value, adding a value with addf")
        self.assertEqual("5", stat.values[1].value, "Bad value, adding a string with add")
        self.assertEqual("00027", stat.values[2].value, "Bad value, adding a string with addf")
        self.assertEqual("toto", stat.values[0].key, "Bad label, adding a value with add")
        self.assertEqual("baba", stat.values[1].key, "Bad label, adding a string with add")
        self.assertEqual("foo", stat.values[2].key, "Bad label, adding a string with addf")

        self.assertEqual("bool", stat.values[3].key, "Bad label, adding a true bool key with add")
        self.assertEqual("True", stat.values[3].value, "Bad value, adding a true bool with add")

        self.assertEqual("bool2", stat.values[4].key, "Bad label, adding a false bool key with add")
        self.assertEqual("False", stat.values[4].value, "Bad value, adding a false bool with add")


    def testFrequencyStatus(self):
        freq_bound = {'min': 10, 'max': 20}

        fs = FrequencyStatus(FrequencyStatusParam(freq_bound, 0.5, 2))

        stat = [DiagnosticStatusWrapper() for i in range(5)]
        fs.tick()
        time.sleep(.02)
        stat[0] = fs.run(stat[0]) # Should be too fast, 20 ms for 1 tick, lower limit should be 33ms.
        time.sleep(.05)
        fs.tick()
        stat[1] = fs.run(stat[1]) # Should be good, 70 ms for 2 ticks, lower limit should be 66 ms.
        time.sleep(.3)
        fs.tick()
        stat[2] = fs.run(stat[2]) # Should be good, 350 ms for 2 ticks, upper limit should be 400 ms.
        time.sleep(.15)
        fs.tick()
        stat[3] = fs.run(stat[3]) # Should be too slow, 450 ms for 2 ticks, upper limit should be 400 ms.
        fs.clear()
        stat[4] = fs.run(stat[4]) # Should be good, just cleared it.

        self.assertEqual(1, stat[0].level, "max frequency exceeded but not reported")
        self.assertEqual(0, stat[1].level, "within max frequency but reported error")
        self.assertEqual(0, stat[2].level, "within min frequency but reported error")
        self.assertEqual(1, stat[3].level, "min frequency exceeded but not reported")
        self.assertEqual(2, stat[4].level, "freshly cleared should fail")
        self.assertEqual("", stat[0].name, "Name should not be set by FrequencyStatus")
        self.assertEqual("Frequency Status", fs.getName(), "Name should be \"Frequency Status\"")


    def testTimeStampStatus(self):
        ts = TimeStampStatus()

        stat = [DiagnosticStatusWrapper() for i in range(5)]
        stat[0] = ts.run(stat[0])
        ts.tick(rospy.Time.now().to_sec() + 2)
        stat[1] = ts.run(stat[1])
        ts.tick(rospy.Time.now())
        stat[2] = ts.run(stat[2])
        ts.tick(rospy.Time.now().to_sec() - 4)
        stat[3] = ts.run(stat[3])
        ts.tick(rospy.Time.now().to_sec() - 6)
        stat[4] = ts.run(stat[4])

        self.assertEqual(1, stat[0].level, "no data should return a warning")
        self.assertEqual(2, stat[1].level, "too far future not reported")
        self.assertEqual(0, stat[2].level, "now not accepted")
        self.assertEqual(0, stat[3].level, "4 seconds ago not accepted")
        self.assertEqual(2, stat[4].level, "too far past not reported")
        self.assertEqual("", stat[0].name, "Name should not be set by TimeStapmStatus")
        self.assertEqual("Timestamp Status", ts.getName(), "Name should be \"Timestamp Status\"")


if __name__ == '__main__':
    rospy.init_node("test_node")
    unittest.main()