#!/usr/bin/env python3
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

from random import random

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
            # Motors
            DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/arms/left/motor', message='OK'),
            DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/arms/right/motor', message='OK'),
            DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/legs/left/motor', message='OK'),
            DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/legs/right/motor', message='OK'),

            # Sensors
            DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/sensors/left/cam', message='OK'),
            DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/sensors/right/cam', message='OK'),
            DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/sensors/front/cam', message='OK'),
            DiagnosticStatus(level=DiagnosticStatus.OK,
                             name='/sensors/rear/cam', message='OK'),
        ]

    def timer_callback(self):
        self.array.header.stamp = ROSClock().now().to_msg()

        # Random diagnostics status
        level = random()
        self.array.status[1].level = DiagnosticStatus.OK
        self.array.status[1].message = 'OK'
        self.array.status[3].level = DiagnosticStatus.OK
        self.array.status[3].message = 'OK'
        self.array.status[4].level = DiagnosticStatus.OK
        self.array.status[4].message = 'OK'
        if level > .5:
            self.array.status[1].level = DiagnosticStatus.WARN
            self.array.status[1].message = 'Warning'
        if level > .7:
            self.array.status[3].level = DiagnosticStatus.WARN
            self.array.status[3].message = 'Warning'
        if level > .95:
            self.array.status[4].level = DiagnosticStatus.ERROR
            self.array.status[4].message = 'Error'

        self.pub.publish(self.array)


def main(args=None):
    rclpy.init(args=args)

    node = DiagnosticTalker()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
