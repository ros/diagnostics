#!/usr/bin/env python3

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

from __future__ import division, with_statement

from io import StringIO
import math
import re
import socket
import subprocess

from diagnostic_msgs.msg import DiagnosticStatus

import diagnostic_updater as DIAG

import rclpy
from rclpy.node import Node


class Sensor(object):

    def __init__(self):
        self.critical = None
        self.min = None
        self.max = None
        self.input = None
        self.name = None
        self.type = None
        self.high = None
        self.alarm = None

    def __repr__(self):
        return 'Sensor object (name: {}, type: {})'.format(self.name, self.type)

    def getCrit(self):
        return self.critical

    def getMin(self):
        return self.min

    def getMax(self):
        return self.max

    def getInput(self):
        return self.input

    def getName(self):
        return self.name

    def getType(self):
        return self.type

    def getHigh(self):
        return self.high

    def getAlarm(self):
        return self.alarm

    def __str__(self):
        lines = []
        lines.append(str(self.name))
        lines.append('\t' + 'Type:  ' + str(self.type))
        if self.input:
            lines.append('\t' + 'Input: ' + str(self.input))
        if self.min:
            lines.append('\t' + 'Min:   ' + str(self.min))
        if self.max:
            lines.append('\t' + 'Max:   ' + str(self.max))
        if self.high:
            lines.append('\t' + 'High:  ' + str(self.high))
        if self.critical:
            lines.append('\t' + 'Crit:  ' + str(self.critical))
        lines.append('\t' + 'Alarm: ' + str(self.alarm))
        return '\n'.join(lines)


def parse_sensor_line(line):
    sensor = Sensor()
    line = line.lstrip()
    [name, reading] = line.split(':')

    try:
        [sensor.name, sensor.type] = name.rsplit(' ', 1)
    except ValueError:
        return None

    if sensor.name == 'Core':
        sensor.name = name
        sensor.type = 'Temperature'
    elif sensor.name.find('Physical id') != -1:
        sensor.name = name
        sensor.type = 'Temperature'

    try:
        [reading, params] = reading.lstrip().split('(')
    except ValueError:
        return None

    sensor.alarm = False
    if line.find('ALARM') != -1:
        sensor.alarm = True

    if reading.find('°C') == -1:
        sensor.input = float(reading.split()[0])
    else:
        sensor.input = float(reading.split('°C')[0])

    params = params.split(',')
    for param in params:
        m = re.search('[0-9]+.[0-9]*', param)
        if param.find('min') != -1:
            sensor.min = float(m.group(0))
        elif param.find('max') != -1:
            sensor.max = float(m.group(0))
        elif param.find('high') != -1:
            sensor.high = float(m.group(0))
        elif param.find('crit') != -1:
            sensor.critical = float(m.group(0))

    return sensor


def _rads_to_rpm(rads):
    return rads / (2 * math.pi) * 60


def _rpm_to_rads(rpm):
    return rpm * (2 * math.pi) / 60


def parse_sensors_output(node: Node, output):
    out = StringIO(output if isinstance(output, str) else output.decode('utf-8'))

    sensorList = []
    for line in out.readlines():
        # Check for a colon
        if ':' in line and 'Adapter' not in line:
            s = None
            try:
                s = parse_sensor_line(line)
            except Exception as exc:
                node.get_logger().warn(
                    'Unable to parse line "%s", due to %s', line, exc
                )
            if s is not None:
                sensorList.append(s)
    return sensorList


def get_sensors():
    p = subprocess.Popen(
        'sensors', stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True
    )
    (o, e) = p.communicate()
    if not p.returncode == 0:
        return ''
    if not o:
        return ''
    return o


class SensorsMonitor(object):

    def __init__(self, node: Node, hostname):
        self.node = node
        self.hostname = hostname
        self.ignore_fans = node.declare_parameter('ignore_fans', False).value
        node.get_logger().info('Ignore fanspeed warnings: %s' % self.ignore_fans)

        self.updater = DIAG.Updater(node)
        self.updater.setHardwareID('none')
        self.updater.add('%s Sensor Status' % self.hostname, self.monitor)

    def monitor(self, stat):
        try:
            stat.summary(DiagnosticStatus.OK, 'OK')
            for sensor in parse_sensors_output(self.node, get_sensors()):
                if sensor.getType() == 'Temperature':
                    if sensor.getInput() > sensor.getCrit():
                        stat.mergeSummary(
                            DiagnosticStatus.ERROR, 'Critical Temperature'
                        )
                    elif sensor.getInput() > sensor.getHigh():
                        stat.mergeSummary(DiagnosticStatus.WARN, 'High Temperature')
                    stat.add(
                        ' '.join([sensor.getName(), sensor.getType()]),
                        str(sensor.getInput()),
                    )
                elif sensor.getType() == 'Voltage':
                    if sensor.getInput() < sensor.getMin():
                        stat.mergeSummary(DiagnosticStatus.ERROR, 'Low Voltage')
                    elif sensor.getInput() > sensor.getMax():
                        stat.mergeSummary(DiagnosticStatus.ERROR, 'High Voltage')
                    stat.add(
                        ' '.join([sensor.getName(), sensor.getType()]),
                        str(sensor.getInput()),
                    )
                elif sensor.getType() == 'Speed':
                    if not self.ignore_fans:
                        if sensor.getInput() < sensor.getMin():
                            stat.mergeSummary(DiagnosticStatus.ERROR, 'No Fan Speed')
                    stat.add(
                        ' '.join([sensor.getName(), sensor.getType()]),
                        str(sensor.getInput()),
                    )
        except Exception:
            import traceback

            self.node.get_logger().error('Unable to process lm-sensors data')
            self.node.get_logger().error(traceback.format_exc())
        return stat


if __name__ == '__main__':
    rclpy.init()
    hostname = socket.gethostname()
    # Every invalid symbol is replaced by underscore.
    # isalnum() alone also allows invalid symbols depending on the locale
    cleaned_hostname = ''.join(
        c if (c.isascii() and c.isalnum()) else '_' for c in hostname)
    node = rclpy.create_node('sensors_monitor_%s' % cleaned_hostname)

    monitor = SensorsMonitor(node, hostname)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
