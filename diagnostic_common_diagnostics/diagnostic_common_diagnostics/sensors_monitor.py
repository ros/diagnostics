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

from io import StringIO
import math
import re
import socket
import subprocess

import diagnostic_updater as DIAG

import rclpy


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

    #  hack for when the name is temp1
    if name.find('temp') != -1:
        return None
    else:
        [sensor.name, sensor.type] = name.rsplit(' ', 1)

    if sensor.name == 'Core':
        sensor.name = name
        sensor.type = 'Temperature'
    elif sensor.name.find('Physical id') != -1:
        sensor.name = name
        sensor.type = 'Temperature'

    [reading, params] = reading.lstrip().split('(')

    sensor.alarm = False
    if line.find('ALARM') != -1:
        sensor.alarm = True

    if reading.find('\xc2\xb0C') == -1:
        sensor.input = float(reading.split()[0])
    else:
        sensor.input = float(reading.split('\xc2\xb0C')[0])

    params = params.split(',')
    for param in params:
        m = re.search('[0-9] + .[0-9]*', param)
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


def parse_sensors_output(output):
    out = StringIO(output)

    sensorList = []
    for line in out.readlines():
        # Check for a colon
        if line.find(':') != -1 and line.find('Adapter') == -1:
            s = parse_sensor_line(line)
            if s is not None:
                sensorList.append(s)
    return sensorList


def get_sensors():
    p = subprocess.Popen('sensors', stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE, shell=True)
    (o, e) = p.communicate()
    if not p.returncode == 0:
        return ''
    if not o:
        return ''
    return o


class SensorsMonitor(object):

    def __init__(self, hostname, node):
        self.hostname = hostname
        self.node = node
        self.ignore_fans = False
        node.get_logger().info('Ignore fanspeed warnings: %s' % self.ignore_fans)

        self.updater = DIAG.Updater(node)
        self.updater.setHardwareID('none')
        self.updater.add('%s Sensor Status' % self.hostname, self.monitor)

        timer_period = 1
        self.timer = self.node.create_timer(timer_period, self.timer_cb)

    def timer_cb(self):
        self.updater.update()

    def monitor(self, stat):
        try:
            stat.summary(DIAG.OK, 'OK')
            for sensor in parse_sensors_output(get_sensors()):
                if sensor.getType() == 'Temperature':
                    if sensor.getInput() > sensor.getCrit():
                        stat.mergeSummary(DIAG.ERROR, 'Critical Temperature')
                    elif sensor.getInput() > sensor.getHigh():
                        stat.mergeSummary(DIAG.WARN, 'High Temperature')
                    stat.add(' '.join([sensor.getName(), sensor.getType()]), sensor.getInput())
                elif sensor.getType() == 'Voltage':
                    if sensor.getInput() < sensor.getMin():
                        stat.mergeSummary(DIAG.ERROR, 'Low Voltage')
                    elif sensor.getInput() > sensor.getMax():
                        stat.mergeSummary(DIAG.ERROR, 'High Voltage')
                    stat.add(' '.join([sensor.getName(), sensor.getType()]), sensor.getInput())
                elif sensor.getType() == 'Speed':
                    if not self.ignore_fans:
                        if sensor.getInput() < sensor.getMin():
                            stat.mergeSummary(DIAG.ERROR, 'No Fan Speed')
                    stat.add(' '.join([sensor.getName(), sensor.getType()]), sensor.getInput())
        except Exception:
            import traceback
            traceback.print_exc()
            #  rospy.logerr('Unable to process lm-sensors data')
            #  rospy.logerr(traceback.format_exc())
        return stat


def main():
    hostname = socket.gethostname()
    rclpy.init()
    node = rclpy.create_node('sensors_monitor_%s' % hostname.replace('-', '_'))
    SensorsMonitor(hostname, node)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
