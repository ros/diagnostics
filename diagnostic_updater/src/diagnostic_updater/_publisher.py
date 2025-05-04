# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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

# -*- coding: utf-8 -*-

""" diagnostic_updater for Python.
@author Brice Rebsamen <brice [dot] rebsamen [gmail]>
"""

import rospy
import threading
from ._update_functions import *


class HeaderlessTopicDiagnostic(CompositeDiagnosticTask):
    """A class to facilitate making diagnostics for a topic using a
    :class:`FrequencyStatus`.

    The word "headerless" in the class name refers to the fact that it is
    mainly designed for use with messages that do not have a header, and
    that cannot therefore be checked using a :class:`TimeStampStatus`.
    """

    def __init__(self, name, diag, freq):
        """Constructs a :class:`HeaderlessTopicDiagnostic`.

        :param str name: The name of the topic that is being diagnosed.

        :param diagnostic_updater.Updater diag: The :class:`Updater` that the :class:`CompositeDiagnosticTask`
                                                should add itself to.

        :param diagnostic_updater.FrequencyStatusParam freq: The parameters for the :class:`FrequencyStatus`
                                                             class that will be computing statistics.
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
        """Clears the frequency statistics."""
        self.freq.clear()


class TopicDiagnostic(HeaderlessTopicDiagnostic):
    """A class to facilitate making diagnostics for a topic using a
    :class:`FrequencyStatus` and :class:`TimeStampStatus`.
    """

    def __init__(self, name, diag, freq, stamp):
        """Constructs a :class:`TopicDiagnostic`.

        :param str name: The name of the topic that is being diagnosed.

        :param diagnostic_updater.Updater diag: The :class:`Updater` that the :class:`CompositeDiagnosticTask`
                                                should add itself to.

        :param diagnostic_updater.FrequencyStatusParam freq: The parameters for the :class:`FrequencyStatus`
                                                             class that will be computing statistics.

        :param diagnostic_updater.TimeStampStatusParam stamp: The parameters for the :class:`TimeStampStatus`
                                                              class that will be computing statistics.
        """

        HeaderlessTopicDiagnostic.__init__(self, name, diag, freq)
        self.stamp = TimeStampStatus(stamp)
        self.addTask(self.stamp)

    def tick(self, stamp):
        """Collects statistics and publishes the message.

        :param stamp: Timestamp to use for interval computation by the
                      :class:`TimeStampStatus` class.
        :type stamp: float or rospy.Time
        """
        self.stamp.tick(stamp)
        HeaderlessTopicDiagnostic.tick(self)


class DiagnosedPublisher(TopicDiagnostic):
    """A :class:`TopicDiagnostic` combined with a :class:`rospy.Publisher`.

    For a standard :class:`rospy.Publisher`, this class allows the :class:`rospy.Publisher` and
    the :class:`TopicDiagnostic` to be combined for added convenience.
    """

    def __init__(self, pub, diag, freq, stamp):
        """Constructs a :class:`DiagnosedPublisher`.

        :param rospy.Publisher pub: The publisher on which statistics are being collected.

        :param diagnostic_updater.Updater diag: The :class:`Updater` that the :class:`CompositeDiagnosticTask`
                                                should add itself to.

        :param diagnostic_updater.FrequencyStatusParam freq: The parameters for the :class:`FrequencyStatus`
                                                             class that will be computing statistics.

        :param diagnostic_updater.TimeStampStatusParam stamp: The parameters for the :class:`TimeStampStatus`
                                                              class that will be computing statistics.
        """
        TopicDiagnostic.__init__(self, pub.name, diag, freq, stamp)
        self.publisher = pub

    def publish(self, message):
        """Collects statistics and publishes the message.

        The timestamp to be used by the :class:`TimeStampStatus` class will be
        extracted from `message.header.stamp`.
        
        :param genpy.Message message: The message to be published.
        """
        self.tick(message.header.stamp)
        self.publisher.publish(message)
