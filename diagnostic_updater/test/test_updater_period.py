#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Tests for the Updater period setter."""

import unittest

from diagnostic_updater import Updater

import rclpy


class TestUpdaterPeriod(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('test_updater_period')

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_period_setter_replaces_timer(self):
        updater = Updater(self.node, period=1.0)
        self.assertEqual(len(list(self.node.timers)), 1)
        old_timer = updater.timer

        updater.period = 0.5

        self.assertEqual(updater.period, 0.5)
        # The old timer must be destroyed rather than left running next to
        # the new one, otherwise update() gets called by both of them.
        self.assertEqual(len(list(self.node.timers)), 1)
        self.assertIsNot(updater.timer, old_timer)
        self.assertEqual(updater.timer.timer_period_ns, 500000000)
