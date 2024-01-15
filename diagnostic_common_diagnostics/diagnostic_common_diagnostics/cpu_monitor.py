#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, TNO IVS, Helmond, Netherlands
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
#  * Neither the name of the TNO IVS nor the names of its
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

# \author Rein Appeldoorn

import collections
import rclpy
import socket
import psutil

from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticTask, Updater


class CpuTask(DiagnosticTask):
    def __init__(self, warning_percentage=90, window=1):
        DiagnosticTask.__init__(self, "CPU Information")

        self._warning_percentage = int(warning_percentage)
        self._readings = collections.deque(maxlen=window)

    def _get_average_reading(self):
        def avg(lst):
            return float(sum(lst)) / len(lst) if lst else float('nan')

        return [avg(cpu_percentages) for cpu_percentages in zip(*self._readings)]

    def run(self, stat):
        self._readings.append(psutil.cpu_percent(percpu=True))
        cpu_percentages = self._get_average_reading()
        cpu_average = sum(cpu_percentages) / len(cpu_percentages)

        stat.add("CPU Load Average", "{:.2f}".format(cpu_average))

        warn = False
        for idx, val in enumerate(cpu_percentages):
            stat.add("CPU {} Load".format(idx), "{:.2f}".format(val))
            if val > self._warning_percentage:
                warn = True

        if warn:
            stat.summary(DiagnosticStatus.WARN,
                         "At least one CPU exceeds {:d} percent".format(self._warning_percentage))
        else:
            stat.summary(DiagnosticStatus.OK, "CPU Average {:.2f} percent".format(cpu_average))

        return stat


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    hostname = socket.gethostname()
    node = Node('cpu_monitor_%s' % hostname.replace("-", "_"))

    # Declare and get parameters
    node.declare_parameter('warning_percentage', 90)
    node.declare_parameter('window', 1)

    warning_percentage = node.get_parameter('warning_percentage').get_parameter_value().integer_value
    window = node.get_parameter('window').get_parameter_value().integer_value

    # Create diagnostic updater with default updater rate of 1 hz
    updater = Updater(node)
    updater.setHardwareID(hostname)
    updater.add(CpuTask(warning_percentage=warning_percentage, window=window))

    rclpy.spin(node)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception:
        import traceback
        traceback.print_exc()
