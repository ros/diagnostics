#! /usr/bin/env python3
# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""
Diagnostic_updater for Python.

@author Brice Rebsamen <brice [dot] rebsamen [gmail]>
"""

# import http.client
import threading

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

import httplib2

# import rclpy
from rclpy.clock import Clock
from rclpy.clock import ClockType
# from rclpy.duration import Duration
# from rclpy.time import Time

from ._diagnostic_status_wrapper import DiagnosticStatusWrapper


class DiagnosticTask:
    """
    DiagnosticTask is an abstract base class for collecting diagnostic data.

    Subclasses are provided for generating common diagnostic information.
    A DiagnosticTask has a name, and a function that is called to cleate a
    DiagnosticStatusWrapper.
    """

    def __init__(self, name):
        """Construct a DiagnosticTask setting its name in the process."""
        self.name = name

    def getName(self):
        """Return the name of the DiagnosticTask."""
        return self.name

    def run(self, stat):
        """
        Fill out this Task's DiagnosticStatusWrapper.

        @param stat: the DiagnosticStatusWrapper to fill
        @return the filled DiagnosticStatusWrapper
        """
        return stat


class FunctionDiagnosticTask(DiagnosticTask):
    """
    A DiagnosticTask based on a function.

    The FunctionDiagnosticTask calls the function when it updates. The
    function updates the DiagnosticStatusWrapper and collects data.

    This is useful for gathering information about a device or driver, like temperature,
    calibration, etc.
    """

    def __init__(self, name, fn):
        """
        Construct a GenericFunctionDiagnosticTask based on the given name and function.

        @param name Name of the function.
        @param fn Function to be called when run is called.
        """
        DiagnosticTask.__init__(self, name)
        self.fn = fn

    def run(self, stat):
        return self.fn(stat)


class CompositeDiagnosticTask(DiagnosticTask):
    """
    Merge CompositeDiagnosticTask into a single DiagnosticTask.

    The CompositeDiagnosticTask allows multiple DiagnosticTask instances to
    be combined into a single task that produces a single single
    DiagnosticStatusWrapped. The output of the combination has the max of
    the status levels, and a concatenation of the non-zero-level messages.

    For instance, this could be used to combine the calibration and offset data
    from an IMU driver.
    """

    def __init__(self, name):
        """Construct a CompositeDiagnosticTask with the given name."""
        DiagnosticTask.__init__(self, name)
        self.tasks = []

    def run(self, stat):
        """Run each child and merges their outputs."""
        combined_summary = DiagnosticStatusWrapper()
        original_summary = DiagnosticStatusWrapper()

        original_summary.summary(stat)

        for task in self.tasks:
            # Put the summary that was passed in.
            stat.summary(original_summary)
            # Let the next task add entries and put its summary.
            stat = task.run(stat)
            # Merge the new summary into the combined summary.
            combined_summary.mergeSummary(stat)
        # Copy the combined summary into the output.
        stat.summary(combined_summary)
        return stat

    def addTask(self, t):
        """
        Add a child CompositeDiagnosticTask.

        This CompositeDiagnosticTask will be called each time this
        CompositeDiagnosticTask is run.
        """
        self.tasks.append(t)


class DiagnosticTaskVector:
    """
    Internal use only.

    Base class for diagnostic_updater::Updater and self_test::Dispatcher.
    The class manages a collection of diagnostic updaters. It contains the
    common functionality used for producing diagnostic updates and for
    self-tests.
    """

    class DiagnosticTaskInternal:
        """Class used to represent a diagnostic task internally in DiagnosticTaskVector."""

        def __init__(self, name, fn):
            self.name = name
            self.fn = fn

        def run(self, stat):
            stat.name = self.name
            return self.fn(stat)

    def __init__(self):
        self.tasks = []
        self.lock = threading.Lock()

    def addedTaskCallback(self, task):
        """
        Allow an action to be taken when a task is added.

        The Updater class
        uses this to immediately publish a diagnostic that says that the node
        is loading.
        """
        pass

    def add(self, *args):
        """
        Add a task to the DiagnosticTaskVector.

        Usage:
        add(task): where task is a DiagnosticTask
        add(name, fn): add a DiagnosticTask embodied by a name and function
        """
        if len(args) == 1:
            task = DiagnosticTaskVector.DiagnosticTaskInternal(args[0].getName(), args[0].run)
        elif len(args) == 2:
            task = DiagnosticTaskVector.DiagnosticTaskInternal(args[0], args[1])

        with self.lock:
            self.tasks.append(task)
            self.addedTaskCallback(task)

    def removeByName(self, name):
        """
        Remove a task based on its name.

        Removes the first task that matches the specified name. (New in
        version 1.1.2)

        @param name Name of the task to remove.
        @return Returns true if a task matched and was removed.
        """
        found = False
        with self.lock:
            for i in range(len(self.tasks)):
                if self.tasks[i].name == name:
                    self.tasks.pop(i)
                    found = True
                    break
        return found


class Updater(DiagnosticTaskVector):
    """
    Manage a list of diagnostic tasks, and calls them in a rate-limited manner.

    This class manages a list of diagnostic tasks. Its update function
    should be called frequently. At some predetermined rate, the update
    function will cause all the diagnostic tasks to run, and will collate
    and publish the resulting diagnostics. The publication rate is
    determined by the "~diagnostic_period" ros parameter.

    The class also allows an update to be forced when something significant
    has happened, and allows a single message to be broadcast on all the
    diagnostics if normal operation of the node is suspended for some
    reason.
    """

    def __init__(self, node):
        """Construct an updater class."""
        DiagnosticTaskVector.__init__(self)
        self.node = node
        self.publisher = self.node.create_publisher(DiagnosticArray, '/diagnostics')
        clock = Clock(clock_type=ClockType.STEADY_TIME)
        now = clock.now()

        self.last_time = now

        self.last_time_period_checked = self.last_time
        self.period = 1

        self.verbose = False
        self.hwid = ''
        self.warn_nohwid_done = False

    def update(self):
        """Causes the diagnostics to update if the inter-update interval has been exceeded."""
        self._check_diagnostic_period()
        clock = Clock(clock_type=ClockType.STEADY_TIME)
        now = clock.now()
        if now >= self.last_time:
            self.force_update()

    def force_update(self):
        """
        Force the diagnostics to update.

        Useful if the node has undergone a drastic state change that should be
        published immediately.
        """
        clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.last_time = clock.now()

        warn_nohwid = len(self.hwid) == 0

        status_vec = []

        with self.lock:  # Make sure no adds happen while we are processing here.
            for task in self.tasks:
                status = DiagnosticStatusWrapper()
                status.level = b'2'
                status.name = task.name
                status.message = 'No message was set'
                status.hardware_id = self.hwid

                status = task.run(status)

                status_vec.append(status)

                if status.level:
                    warn_nohwid = False

                if self.verbose and status.level:
                    self.node.get_logger().warn('Non-zero diagnostic status. Name: %s, status\
                                                %i: %s' % (status.name, status.level,
                                                           status.message))

        if warn_nohwid and not self.warn_nohwid_done:
            self.node.get_logger().warn('diagnostic_updater: No HW_ID was set. This is probably\
                                        a bug. Please report it. For devices that do not have a\
                                        HW_ID, set this value to none. This warning only occurs\
                                        once all diagnostics are OK so it is okay to wait until\
                                        the device is open before calling setHardwareID.')
            self.warn_nohwid_done = True

        self.publish(status_vec)

    def broadcast(self, lvl, msg):
        """
        Output a message on all the known DiagnosticStatus.

        Useful if something drastic is happening such as shutdown or a self-test.

        @param lvl Level of the diagnostic being output.
        @param msg Status message to output.
        """
        status_vec = []

        for task in self.tasks:
            status = DiagnosticStatusWrapper()
            status.name = task.name
            status.summary(lvl, msg)
            status_vec.append(status)

        self.publish(status_vec)

    def setHardwareID(self, hwid):
        self.hwid = hwid

    def _check_diagnostic_period(self):
        """Recheck the diagnostic_period on the parameter server."""
        # This was getParamCached() call in the cpp code. i.e. it would throttle
        # the actual call to the parameter server using a notification of change
        # mechanism.
        # This is not available in rospy. Hence I throttle the call to the
        # parameter server using a standard timeout mechanism (4Hz)
        clock = Clock(clock_type=ClockType.STEADY_TIME)
        now = clock.now()
        if now >= self.last_time_period_checked:
            try:
                self.last_time_period_checked = now
            except (httplib2.CannotSendRequest, httplib2.ResponseNotReady):
                pass

    def publish(self, msg):
        """Publish a single diagnostic status or a vector of diagnostic statuses."""
        if not type(msg) is list:
            msg = [msg]

        for stat in msg:
            stat.name = self.node.get_name() + ': ' + stat.name
        clock = Clock(clock_type=ClockType.STEADY_TIME)
        now = clock.now()

        da = DiagnosticArray()
        db = DiagnosticStatus()
        db.name = stat.name
        db.message = stat.message
        db.hardware_id = stat.hardware_id
        db.values = stat.values
        da.status.append(db)
        da.header.stamp = now.to_msg()  # Add timestamp for ROS 0.10
        self.publisher.publish(da)

    def addedTaskCallback(self, task):
        stat = DiagnosticStatusWrapper()
        stat.name = task.name
        stat.summary(b'0', 'Node starting up')
        self.publish(stat)
