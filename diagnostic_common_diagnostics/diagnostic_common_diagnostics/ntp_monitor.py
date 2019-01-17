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

import re
import socket
from subprocess import PIPE, Popen
import sys
import threading

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import rclpy
from rclpy.clock import Clock
from rclpy.clock import ClockType


def ntp_diag(st, host, off, error_offset):

    try:
        print(host)
        p = Popen(['ntpdate', '-q', host], stdout=PIPE, stdin=PIPE, stderr=PIPE)
        res = p.wait()
        (o, e) = p.communicate()
    except OSError as errno:
        if errno == 4:
            return None  # ctrl-c interrupt
        else:
            raise
    if (res == 0):
        measured_offset = float(re.search(b'offset (.*),', o).group(1)) * 1000000

        st.level = DiagnosticStatus.OK
        st.message = 'OK'
        st.values.insert(0, KeyValue(key='Offset (us)', value=str(measured_offset)))
        st.values.insert(1, KeyValue(key='Offset tolerance (us)', value=str(off)))
        st.values.insert(2, KeyValue(key='Offset tolerance (us) for Error',
                         value=str(error_offset)))
        if (abs(measured_offset) > off):
            st.level = DiagnosticStatus.WARN
            st.message = 'NTP Offset Too High'
        if (abs(measured_offset) > error_offset):
            st.level = DiagnosticStatus.ERROR
            st.message = 'NTP Offset Too High'

    else:
        st.level = DiagnosticStatus.ERROR
        st.message = 'Error Running ntpdate. Returned %d' % res
        st.values.insert(0, KeyValue(key='Offset (us)', value=str('N/A')))
        st.values.insert(1, KeyValue(key='Offset tolerance (us)', value=str(off)))
        st.values.insert(2, KeyValue(key='Offset tolerance (us) for Error',
                         value=str(error_offset)))
        st.values.insert(3, KeyValue(key='Output', value='o'))
        st.values.insert(4, KeyValue(key='Errors', value='e'))
    return st


class NTPMonitor:

    def __init__(self, ntp_hostname, node, offset=500, self_offset=500,
                 diag_hostname=None, error_offset=5000000,
                 do_self_test=True):

        self.ntp_hostname = ntp_hostname
        self.offset = offset
        self.self_offset = self_offset
        self.diag_hostname = diag_hostname
        self.error_offset = error_offset
        self.do_self_test = do_self_test
        self.hostname = socket.gethostname()
        if self.diag_hostname is None:
            self.diag_hostname = self.hostname

        self.stat = DiagnosticStatus()
        self.stat.level = DiagnosticStatus.OK
        self.stat.name = 'NTP offset from ' + self.diag_hostname + ' to ' + self.ntp_hostname
        self.stat.message = 'OK'
        self.stat.hardware_id = self.hostname
        self.stat.values = []

        self.self_stat = DiagnosticStatus()
        self.self_stat.level = DiagnosticStatus.OK
        self.self_stat.name = 'NTP self-offset for ' + self.diag_hostname
        self.self_stat.message = 'OK'
        self.self_stat.hardware_id = self.hostname
        self.self_stat.values = []

        self.mutex = threading.Lock()
        self.node = node
        self.pub = self.node.create_publisher(DiagnosticArray, '/diagnostics')

        #  we need to periodically republish this
        self.current_msg = None
        timer_period = 1.0
        self.pubtimer = self.node.create_timer(timer_period, self.pubCB)
        self.checktimer = self.node.create_timer(timer_period, self.checkCB)

    def pubCB(self):
        with self.mutex:
            if self.current_msg:
                print(self.current_msg)
                self.pub.publish(self.current_msg)

    def checkCB(self):
        self.checktimer.cancel()
        new_msg = DiagnosticArray()
        clock = Clock(clock_type=ClockType.STEADY_TIME)
        now = clock.now()
        new_msg.header.stamp = now.to_msg()
        st = ntp_diag(self.stat, self.ntp_hostname, self.offset, self.error_offset)
        if st is not None:
            new_msg.status.append(st)

        if self.do_self_test:
            st = ntp_diag(self.self_stat, self.hostname, self.self_offset, self.error_offset)
            if st is not None:
                new_msg.status.append(st)

        with self.mutex:
            self.current_msg = new_msg

        timer_period = 10
        self.checktimer = self.node.create_timer(timer_period, self.checkCB)


def ntp_monitor_main(node, argv=sys.argv):
    import optparse
    parser = optparse.OptionParser(usage='usage: ntp_monitor ntp-hostname []')
    parser.add_option('--offset-tolerance', dest='offset_tol',
                      action='store', default=500,
                      help='Offset from NTP host', metavar='OFFSET-TOL')
    parser.add_option('--error-offset-tolerance', dest='error_offset_tol',
                      action='store', default=5000000,
                      help='Offset from NTP host. Above this is error', metavar='OFFSET-TOL')
    parser.add_option('--self_offset-tolerance', dest='self_offset_tol',
                      action='store', default=500,
                      help='Offset from self', metavar='SELF_OFFSET-TOL')
    parser.add_option('--diag-hostname', dest='diag_hostname',
                      help='Computer name in diagnostics output (ex: "c1")',
                      metavar='DIAG_HOSTNAME',
                      action='store', default=None)
    parser.add_option('--no-self-test', dest='do_self_test',
                      help='Disable self test',
                      action='store_false', default=True)
    options, args = parser.parse_args()

    if (len(args) != 2):
        parser.error('Invalid arguments. Must have HOSTNAME [args]. %s' % args)

    try:
        offset = int(options.offset_tol)
        self_offset = int(options.self_offset_tol)
        error_offset = int(options.error_offset_tol)
    except Exception:
        parser.error('Offsets must be numbers')

    NTPMonitor(args[1], node, offset, self_offset,
               options.diag_hostname, error_offset,
               options.do_self_test)
    rclpy.spin(node)


def main():
    rclpy.init()
    node = rclpy.create_node('ntp_monitor')
    try:
        ntp_monitor_main(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    except Exception:
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
