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

import socket
from time import sleep

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticTask, Updater
import psutil

import rclpy
from rclpy.parameter import Parameter


class CpuTask(DiagnosticTask):

    def __init__(self, warning_percentage):
        DiagnosticTask.__init__(self, 'CPU Information')
        self._warning_percentage = int(warning_percentage)

    def run(self, stat):
        cpu_percentages = psutil.cpu_percent(percpu=True)
        cpu_average = sum(cpu_percentages) / len(cpu_percentages)
        warn = False
        for idx, val in enumerate(cpu_percentages):
            stat.add('CPU {} Load'.format(idx), '{}'.format(val))
            if val > self._warning_percentage:
                warn = True

        if warn:
            stat.summary(DiagnosticStatus.WARN,
                         'At least one CPU exceeds %d percent' % self._warning_percentage)
        else:
            stat.summary(DiagnosticStatus.OK, 'CPU Average %.1f percent' % cpu_average)

        return stat


def main():
    hostname = socket.gethostname()
    rclpy.init()
    node = rclpy.create_node('cpu_monitor_%s' % hostname.replace('-', '_'))
    Parameter('~warning_percentage', Parameter.Type.INTEGER, 90)

    updater = Updater(node)
    updater.setHardwareID(hostname)
    updater.add(CpuTask(90))

    while rclpy.ok():
        sleep(1)
        updater.update()


if __name__ == '__main__':
    main()
