#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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
#

# \author Kevin Watts
# \brief Publishes diagnostic messages for diagnostic aggregator unit test

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
from rclpy.clock import ROSClock
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

PKG = 'diagnostic_aggregator'


class DiagnosticTalker(Node):

    def __init__(self):
        super().__init__('diagnostic_talker')
        self.i = 0
        self.pub = self.create_publisher(DiagnosticArray,
                                         '/diagnostics',
                                         qos_profile_system_default)
        timer_period = 1.0
        self.tmr = self.create_timer(timer_period, self.timer_callback)

        self.array = DiagnosticArray()
        self.array.status = [
            # GenericAnalyzer prefix1
            DiagnosticStatus(
                level=DiagnosticStatus.OK,
                name='pref1a',
                message='OK'),
            DiagnosticStatus(
                level=DiagnosticStatus.WARN,
                name='pref1b',
                message='Warning'),
            DiagnosticStatus(
                level=DiagnosticStatus.OK,
                name='contains1a',
                message='OK'),
            DiagnosticStatus(
                level=DiagnosticStatus.OK,
                name='prefix1: contains1b',
                message='OK'),
            DiagnosticStatus(
                level=DiagnosticStatus.OK,
                name='name1',
                message='OK'),
            DiagnosticStatus(
                level=DiagnosticStatus.OK,
                name='prefix1: expected1a',
                message='OK'),
            DiagnosticStatus(
                level=DiagnosticStatus.OK,
                name='prefix1: expected1b',
                message='OK'),
            DiagnosticStatus(
                level=DiagnosticStatus.OK,
                name='prefix1: expected1c',
                message='OK'),
            DiagnosticStatus(
                level=DiagnosticStatus.OK,
                name='prefix1: expected1d',
                message='OK'),
            DiagnosticStatus(
                level=DiagnosticStatus.OK,
                name='find1_items: find_remove1a',
                message='OK'),
            DiagnosticStatus(
                level=DiagnosticStatus.OK,
                name='find1_items: find_remove1b',
                message='OK'),

            # GenericAnalyzer prefix2
            DiagnosticStatus(
                level=DiagnosticStatus.OK,
                name='contain2a',
                message='OK'),
            DiagnosticStatus(
                level=DiagnosticStatus.OK,
                name='contain2b',
                message='OK'),
            DiagnosticStatus(
                level=DiagnosticStatus.OK,
                name='name2',
                message='OK'),

            # OtherAnalyzer for Other
            DiagnosticStatus(
                level=DiagnosticStatus.ERROR,
                name='other1',
                message='Error'),
            DiagnosticStatus(
                level=DiagnosticStatus.OK,
                name='other2',
                message='OK'),
            DiagnosticStatus(
                level=DiagnosticStatus.OK,
                name='other3',
                message='OK')]

    def timer_callback(self):
        self.array.header.stamp = ROSClock().now().to_msg()
        self.pub.publish(self.array)


def main(args=None):
    rclpy.init(args=args)

    node = DiagnosticTalker()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
