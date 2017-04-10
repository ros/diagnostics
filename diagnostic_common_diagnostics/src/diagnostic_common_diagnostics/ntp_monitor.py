#!/usr/bin/env python
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

import roslib
roslib.load_manifest('diagnostic_common_diagnostics')
import rospy
import diagnostic_updater as DIAG

import sys
import threading
import socket
from subprocess import Popen, PIPE
import time
import re


def ntp_diag(st, host, off, error_offset):
    try:
        p = Popen(["ntpdate", "-q", host], stdout=PIPE, stdin=PIPE, stderr=PIPE)
        res = p.wait()
        (o,e) = p.communicate()
    except OSError, (errno, msg):
        if errno == 4:
            return None #ctrl-c interrupt
        else:
            raise
    if (res == 0):
        measured_offset = float(re.search("offset (.*),", o).group(1))*1000000

        st.level = DIAG.DiagnosticStatus.OK
        st.message = "OK"
        st.values = [ DIAG.KeyValue("Offset (us)", str(measured_offset)),
                        DIAG.KeyValue("Offset tolerance (us)", str(off)),
                        DIAG.KeyValue("Offset tolerance (us) for Error", str(error_offset)) ]

        if (abs(measured_offset) > off):
            st.level = DIAG.DiagnosticStatus.WARN
            st.message = "NTP Offset Too High"
        if (abs(measured_offset) > error_offset):
            st.level = DIAG.DiagnosticStatus.ERROR
            st.message = "NTP Offset Too High"

    else:
        st.level = DIAG.DiagnosticStatus.ERROR
        st.message = "Error Running ntpdate. Returned %d" % res
        st.values = [ DIAG.KeyValue("Offset (us)", "N/A"),
                        DIAG.KeyValue("Offset tolerance (us)", str(off)),
                        DIAG.KeyValue("Offset tolerance (us) for Error", str(error_offset)),
                        DIAG.KeyValue("Output", o),
                        DIAG.KeyValue("Errors", e) ]

    return st


class NTPMonitor:
    
    def __init__(self, ntp_hostname, offset=500, self_offset=500,
                 diag_hostname = None, error_offset = 5000000,
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

        self.stat = DIAG.DiagnosticStatus()
        self.stat.level = DIAG.DiagnosticStatus.OK
        self.stat.name = "NTP offset from "+ self.diag_hostname + " to " + self.ntp_hostname
        self.stat.message = "OK"
        self.stat.hardware_id = self.hostname
        self.stat.values = []

        self.self_stat = DIAG.DiagnosticStatus()
        self.self_stat.level = DIAG.DiagnosticStatus.OK
        self.self_stat.name = "NTP self-offset for "+ self.diag_hostname
        self.self_stat.message = "OK"
        self.self_stat.hardware_id = self.hostname
        self.self_stat.values = []

        self.mutex = threading.Lock()
        self.pub = rospy.Publisher("/diagnostics", DIAG.DiagnosticArray, queue_size=10)

        # we need to periodically republish this
        self.current_msg = None
        self.pubtimer = rospy.Timer(rospy.Duration(.1), self.pubCB)
        self.checktimer = rospy.Timer(rospy.Duration(.1), self.checkCB, True)

    def pubCB(self, ev):
        with self.mutex:
            if self.current_msg:
                self.pub.publish(self.current_msg)

    def checkCB(self, ev):
        new_msg = DIAG.DiagnosticArray()
        new_msg.header.stamp = rospy.get_rostime()

        st = ntp_diag(self.stat, self.ntp_hostname, self.offset, self.error_offset)
        if st is not None:
            new_msg.status.append(st)

        if self.do_self_test:
            st = ntp_diag(self.self_stat, self.hostname, self.self_offset, self.error_offset)
            if st is not None:
                new_msg.status.append(st)

        with self.mutex:
            self.current_msg = new_msg

        self.checktimer = rospy.Timer(rospy.Duration(10), self.checkCB, True)


def ntp_monitor_main(argv=sys.argv):
    import optparse
    parser = optparse.OptionParser(usage="usage: ntp_monitor ntp-hostname []")
    parser.add_option("--offset-tolerance", dest="offset_tol",
                      action="store", default=500,
                      help="Offset from NTP host", metavar="OFFSET-TOL")
    parser.add_option("--error-offset-tolerance", dest="error_offset_tol",
                      action="store", default=5000000,
                      help="Offset from NTP host. Above this is error", metavar="OFFSET-TOL")
    parser.add_option("--self_offset-tolerance", dest="self_offset_tol",
                      action="store", default=500,
                      help="Offset from self", metavar="SELF_OFFSET-TOL")
    parser.add_option("--diag-hostname", dest="diag_hostname",
                      help="Computer name in diagnostics output (ex: 'c1')",
                      metavar="DIAG_HOSTNAME",
                      action="store", default=None)
    parser.add_option("--no-self-test", dest="do_self_test",
                      help="Disable self test",
                      action="store_false", default=True)
    options, args = parser.parse_args(rospy.myargv())

    if (len(args) != 2):
        parser.error("Invalid arguments. Must have HOSTNAME [args]. %s" % args)


    try:
        offset = int(options.offset_tol)
        self_offset = int(options.self_offset_tol)
        error_offset = int(options.error_offset_tol)
    except:
        parser.error("Offsets must be numbers")

    ntp_monitor = NTPMonitor(args[1], offset, self_offset,
                             options.diag_hostname, error_offset,
                             options.do_self_test)

    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("ntp_monitor", anonymous=True)
    try:
        ntp_monitor_main(rospy.myargv())
    except KeyboardInterrupt: pass
    except SystemExit: pass
    except:
        import traceback
        traceback.print_exc()
