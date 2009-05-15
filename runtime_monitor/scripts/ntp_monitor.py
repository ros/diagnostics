#!/usr/bin/env python
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
roslib.load_manifest('runtime_monitor')

from robot_msgs.msg import *
import sys
import rospy
import socket
from subprocess import Popen, PIPE

import time

import re

NAME = 'ntp_monitor'

def ntp_monitor(ntp_hostname, offset=500):
    pub = rospy.Publisher("/diagnostics", DiagnosticMessage)
    rospy.init_node(NAME, anonymous=True)

    hostname = socket.gethostbyaddr(socket.gethostname())[0]

    while not rospy.is_shutdown():
        try:
            p = Popen(["ntpdate", "-q", ntp_hostname], stdout=PIPE, stdin=PIPE, stderr=PIPE)
            res = p.wait()
            (o,e) = p.communicate()
        except OSError, (errno, msg):
            if errno == 4:
                break #ctrl-c interrupt
            else:
                raise
        if (res == 0):
            measured_offset = float(re.search("offset (.*),", o).group(1))*1000000
            if (abs(measured_offset) < offset):
                stat = DiagnosticStatus(0,"NTP offset from: "+ hostname + " to: " +ntp_hostname, "Acceptable synchronization", [DiagnosticValue(measured_offset,"offset (us)")],[])
                print measured_offset
            else:
                stat = DiagnosticStatus(2,"NTP offset from: "+ hostname + " to: " +ntp_hostname, "Offset too great", [DiagnosticValue(measured_offset,"offset (us)")],[])
        else:
            stat = DiagnosticStatus(2,"NTP offset from: "+ hostname + " to: " +ntp_hostname, "Failure to run ntpdate -q", [],[])

        pub.publish(DiagnosticMessage(None, [stat]))
        time.sleep(1)

def ntp_monitor_main(argv=sys.argv):
    import optparse
    parser = optparse.OptionParser(usage="usage: ntp_monitor ntp-hostname [offset-tolerance]")
    options, args = parser.parse_args(argv[1:])
    if len(args) == 1:
        ntp_hostname, offset = args[0], 500
    elif len(args) == 2:
        ntp_hostname, offset = args
        try:
            offset = int(offset)
        except:
            parser.error("offset must be a number")
    else:
        parser.error("Invalid arguments")
    ntp_monitor(ntp_hostname, offset)

if __name__ == "__main__":
    try:
        ntp_monitor_main(rospy.myargv())
    except KeyboardInterrupt: pass
    
