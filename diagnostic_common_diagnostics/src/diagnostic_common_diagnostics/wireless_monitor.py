#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Eurotec, Netherlands
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
#  * Neither the name of Eurotec nor the names of its
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
import re
import subprocess

import rospy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticTask, Updater


class WirelessTask(DiagnosticTask):
    def __init__(self, interface, link_quality_warning_percentage):
        DiagnosticTask.__init__(self, "Wireless Information")
        self._interface = interface
        self._signal_quality_warning_percentage = int(link_quality_warning_percentage)

    @staticmethod
    def _execute_command(cmd):
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        stdout, stderr = p.communicate()
        if p.returncode != 0:
            raise RuntimeError("Could not execute command {}".format(cmd))
        return stdout

    def _update_with_iwconfig_status(self, stat):
        def parse_iwconfig_output(output):
            result = {}
            items = [e.strip() for e in output.replace('\n', '  ').split('  ') if ":" in e or "=" in e]
            for item in items:
                key, value = re.search('(.+?)[:=](.+)', item).groups()
                result[key] = value.strip().strip("\"")
            return result

        output = self._execute_command('iwconfig {}'.format(self._interface))
        result = parse_iwconfig_output(output)

        if 'Link Quality' in result:
            # Rewrite Link Quality because we want to do logic on this variable
            numerator, denominator = result['Link Quality'].split("/")
            link_quality_percentage = float(numerator) / float(denominator)
            result['Link Quality'] = link_quality_percentage

            if link_quality_percentage > self._signal_quality_warning_percentage:
                stat.summary(DiagnosticStatus.OK, "Link quality OK")
            else:
                stat.summary(DiagnosticStatus.WARN, "Link quality low")
        else:
            stat.summary(DiagnosticStatus.ERROR, "Link quality unknown")

        for k, v in result.iteritems():
            stat.add(k, v)

    def _update_with_sar_status(self, stat):
        output = self._execute_command('sar -n DEV 1 1'.format(self._interface))
        headers = re.findall('[^ ]+', re.search('Average:.+IFACE(.+?)\n', output).group(1))
        values = re.findall('[^ ]+', re.search('Average:.+{}(.+?)\n'.format(self._interface), output).group(1))
        for k, v in zip(headers, values):
            stat.add(k, v)

    def run(self, stat):
        try:
            self._update_with_iwconfig_status(stat)
            self._update_with_sar_status(stat)
        except RuntimeError as e:
            stat.summary(DiagnosticStatus.ERROR, "Failed to update wireless information: {}".format(e))
        return stat


def main():
    rospy.init_node('wireless_monitor')
    interface = rospy.get_param('~interface', 'wlo1')

    updater = Updater()
    updater.setHardwareID(interface)
    updater.add(WirelessTask(
        interface,
        rospy.get_param("~link_quality_warning_percentage", 0.5)
    ))

    rate = rospy.Rate(rospy.get_param("~rate", 1))
    while not rospy.is_shutdown():
        updater.force_update()  # We want do determine the rate ourselves (not the internal rate of the updater)
        rate.sleep()


if __name__ == '__main__':
    main()
