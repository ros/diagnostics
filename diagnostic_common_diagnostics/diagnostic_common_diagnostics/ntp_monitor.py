#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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

import socket
import sys
import threading

import diagnostic_updater as DIAG

import ntplib

import rclpy
from rclpy.node import Node


class NTPMonitor(Node):
    """A diagnostic task that monitors the NTP offset of the system clock."""

    def __init__(self, ntp_hostname, offset=500, self_offset=500,
                 diag_hostname=None, error_offset=5000000,
                 do_self_test=True):
        """Initialize the NTPMonitor."""
        super().__init__(__class__.__name__)

        self.ntp_hostname = ntp_hostname
        self.offset = offset
        self.self_offset = self_offset
        self.diag_hostname = diag_hostname
        self.error_offset = error_offset
        self.do_self_test = do_self_test

        self.hostname = socket.gethostname()
        if self.diag_hostname is None:
            self.diag_hostname = self.hostname

        self.stat = DIAG.DiagnosticStatus()
        self.stat.level = DIAG.DiagnosticStatus.OK
        self.stat.name = 'NTP offset from ' + \
            self.diag_hostname + ' to ' + self.ntp_hostname
        self.stat.message = 'OK'
        self.stat.hardware_id = self.hostname
        self.stat.values = []

        self.self_stat = DIAG.DiagnosticStatus()
        self.self_stat.level = DIAG.DiagnosticStatus.OK
        self.self_stat.name = 'NTP self-offset for ' + self.diag_hostname
        self.self_stat.message = 'OK'
        self.self_stat.hardware_id = self.hostname
        self.self_stat.values = []

        self.mutex = threading.Lock()
        self.pub = self.create_publisher(
            DIAG.DiagnosticArray, '/diagnostics', 10)

        # we need to periodically republish this
        self.current_msg = None
        self.pubtimer = self.create_timer(0.1, self.pubCB)
        self.checktimer = self.create_timer(0.1, self.checkCB)

    def pubCB(self):
        with self.mutex:
            if self.current_msg:
                self.pub.publish(self.current_msg)

    def checkCB(self):
        new_msg = DIAG.DiagnosticArray()

        st = self.ntp_diag(self.stat)
        if st is not None:
            new_msg.status.append(st)

        if self.do_self_test:
            st = self.ntp_diag(self.self_stat)
            if st is not None:
                new_msg.status.append(st)

        with self.mutex:
            self.current_msg = new_msg

    def ntp_diag(self, st):
        """
        Add ntp diagnostics to the given status message and return it.

        Args:
        ----
            st: The diagnostic status object to populate

        """

        def add_kv(stat_values, key, value):
            kv = DIAG.KeyValue()
            kv.key = key
            kv.value = value
            stat_values.append(kv)

        ntp_client = ntplib.NTPClient()
        response = None
        try:
            response = ntp_client.request(self.ntp_hostname, version=3)
        except ntplib.NTPException as e:
            self.get_logger().error(f'NTP Error: {e}')
            st.level = DIAG.DiagnosticStatus.ERROR
            st.message = f'NTP Error: {e}'
            add_kv(st.values, 'Offset (us)', 'N/A')
            add_kv(st.values, 'Errors', str(e))

        if response is not None:
            measured_offset = response.offset * 1e6
            if (abs(measured_offset) > self.offset):
                st.level = DIAG.DiagnosticStatus.WARN
                st.message = \
                    f'NTP offset above threshold: {measured_offset}>'\
                    f'{self.offset} us'
            if (abs(measured_offset) > self.error_offset):
                st.level = DIAG.DiagnosticStatus.ERROR
                st.message = \
                    f'NTP offset above error threshold: {measured_offset}>'\
                    f'{self.error_offset} us'
            if (abs(measured_offset) < self.offset):
                st.level = DIAG.DiagnosticStatus.OK
                st.message = f'NTP Offset OK: {measured_offset} us'

        return st


def ntp_monitor_main(argv=sys.argv[1:]):
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--ntp_hostname',
                        action='store', default='0.pool.ntp.org',
                        type=str)
    parser.add_argument('--offset-tolerance', dest='offset_tol',
                        action='store', default=500,
                        help='Offset from NTP host [us]', metavar='OFFSET-TOL',
                        type=int)
    parser.add_argument('--error-offset-tolerance', dest='error_offset_tol',
                        action='store', default=5000000,
                        help='Offset from NTP host. Above this is error',
                        metavar='OFFSET-TOL', type=int)
    parser.add_argument('--self_offset-tolerance', dest='self_offset_tol',
                        action='store', default=500,
                        help='Offset from self [us]', metavar='SELF_OFFSET-TOL',
                        type=int)
    parser.add_argument('--diag-hostname', dest='diag_hostname',
                        help='Computer name in diagnostics output (ex: "c1")',
                        metavar='DIAG_HOSTNAME',
                        action='store', default=None,
                        type=str)
    parser.add_argument('--no-self-test', dest='do_self_test',
                        help='Disable self test',
                        action='store_false', default=True)
    args = parser.parse_args(args=argv)

    offset = args.offset_tol
    self_offset = args.self_offset_tol
    error_offset = args.error_offset_tol
    assert offset < error_offset, \
        'Offset tolerance must be less than error offset tolerance'

    ntp_monitor = NTPMonitor(args.ntp_hostname, offset, self_offset,
                             args.diag_hostname, error_offset,
                             args.do_self_test)

    rclpy.spin(ntp_monitor)


def main(args=None):
    rclpy.init(args=args)
    try:
        ntp_monitor_main()
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    except Exception:
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
