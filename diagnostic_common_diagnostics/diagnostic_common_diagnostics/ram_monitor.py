#!/usr/bin/env python3
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
import socket

from diagnostic_msgs.msg import DiagnosticStatus

from diagnostic_updater import DiagnosticTask, Updater

import psutil

import rclpy


class RamTask(DiagnosticTask):

    def __init__(self, warning_percentage, window):
        DiagnosticTask.__init__(self, 'RAM Information')
        self._warning_percentage = int(warning_percentage)
        self._readings = collections.deque(maxlen=window)

    def run(self, stat):
        self._readings.append(psutil.virtual_memory().percent)
        ram_average = sum(self._readings) / len(self._readings)

        stat.add('RAM Load Average', f'{ram_average:.2f}')

        if ram_average > self._warning_percentage:
            stat.summary(
                DiagnosticStatus.WARN,
                f'RAM Average exceeds {self._warning_percentage:d} percent',
            )
        else:
            stat.summary(DiagnosticStatus.OK, f'RAM Average {ram_average:.2f} percent')

        return stat


def main():
    hostname = socket.gethostname()
    # Every invalid symbol is replaced by underscore.
    # isalnum() alone also allows invalid symbols depending on the locale
    cleaned_hostname = ''.join(
        c if (c.isascii() and c.isalnum()) else '_' for c in hostname)
    rclpy.init()
    node = rclpy.create_node(f'ram_monitor_{cleaned_hostname}')

    updater = Updater(node)
    updater.setHardwareID(hostname)
    updater.add(
        RamTask(
            node.declare_parameter('warning_percentage', 90).value,
            node.declare_parameter('window', 1).value,
        )
    )

    rclpy.spin(node)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
