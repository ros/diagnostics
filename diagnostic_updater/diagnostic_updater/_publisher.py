#! /usr/bin/env python3
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

"""
Diagnostic_updater for Python.

@author Brice Rebsamen <brice [dot] rebsamen [gmail]>
"""
import rclpy
import threading
from ._update_functions import *


class HeaderlessTopicDiagnostic(CompositeDiagnosticTask):
    """
    A class to facilitate making diagnostics for a topic using a FrequencyStatus.

    The word "headerless" in the class name refers to the fact that it is
    mainly designed for use with messages that do not have a header, and
    that cannot therefore be checked using a TimeStampStatus.
    """

    def __init__(self, name, diag, freq):
        """
        Construct a HeaderlessTopicDiagnostic.

        @param name The name of the topic that is being diagnosed.

        @param diag The diagnostic_updater that the CompositeDiagnosticTask
        should add itself to.

        @param freq The parameters for the FrequencyStatus class that will be
        computing statistics.
        """
        CompositeDiagnosticTask.__init__(self, name + " topic status")
        self.diag = diag
        self.freq = FrequencyStatus(freq)
        self.addTask(self.freq)
        self.diag.add(self)

    def tick(self):
        """Signals that a publication has occurred."""
        self.freq.tick()

    def clear_window(self):
        """Clear the frequency statistics."""
        self.freq.clear()


class TopicDiagnostic(HeaderlessTopicDiagnostic):
    """
    A class to facilitate making diagnostics for a topic using.
    
    a FrequencyStatus and TimeStampStatus.
    """

    def __init__(self, name, diag, freq, stamp):
        """
        Construct a TopicDiagnostic.

        @param name The name of the topic that is being diagnosed.

        @param diag The diagnostic_updater that the CompositeDiagnosticTask
        should add itself to.

        @param freq The parameters for the FrequencyStatus class that will be
        computing statistics.

        @param stamp The parameters for the TimeStampStatus class that will be
        computing statistics.
        """
        HeaderlessTopicDiagnostic.__init__(self, name, diag, freq)
        self.stamp = TimeStampStatus(stamp)
        self.addTask(self.stamp)

    def tick(self, stamp):
        """
        Collect statistics and publishes the message.

        @param stamp Timestamp to use for interval computation by the
        TimeStampStatus class.
        """
        self.stamp.tick(stamp)
        HeaderlessTopicDiagnostic.tick(self)


class DiagnosedPublisher(TopicDiagnostic):
    """
    A TopicDiagnostic combined with a ros::Publisher.

    For a standard ros::Publisher, this class allows the ros::Publisher and
    the TopicDiagnostic to be combined for added convenience.
    """

    def __init__(self, pub, diag, freq, stamp):
        """
        Construct a DiagnosedPublisher.

        @param pub The publisher on which statistics are being collected.

        @param diag The diagnostic_updater that the CompositeDiagnosticTask
        should add itself to.

        @param freq The parameters for the FrequencyStatus class that will be
        computing statistics.

        @param stamp The parameters for the TimeStampStatus class that will be
        computing statistics.
        """
        TopicDiagnostic.__init__(self, pub.name, diag, freq, stamp)
        self.publisher = pub

    def publish(self, message):
        """
        Collect statistics and publishes the message.

        The timestamp to be used by the TimeStampStatus class will be
        extracted from message.header.stamp.
        """
        self.tick(message.header.stamp)
        self.publisher.publish(message)
