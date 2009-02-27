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
roslib.load_manifest('ntp_monitor')

from robot_msgs.msg import *
import sys
import rospy
import socket
import subprocess

import time

import re

NAME = 'ntp_monitor'

def ntp_monitor():
    if len(sys.argv) < 2:
        print "Please pass host as argument"
        return
    else:
        pub = rospy.Publisher("/diagnostics", DiagnosticMessage)
        rospy.init_node(NAME, anonymous=True)

        hostname = socket.gethostbyaddr(socket.gethostname())[0]
  
        while not rospy.is_shutdown():
            p = subprocess.Popen(["ntpdate", "-q", sys.argv[1]], stdout=subprocess.PIPE, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
            res = p.wait()
            (o,e) = p.communicate()
            if (res == 0):
                offset = float(re.search("offset (.*),", o).group(1))*1000000
                if (abs(offset) < 500):
                    stat = DiagnosticStatus(0,hostname + " NTP offset", "Acceptable synchronization", [DiagnosticValue(offset,"offset (us)")],[])
                    print offset
                else:
                    stat = DiagnosticStatus(2,hostname + " NTP offset", "Offset too great", [DiagnosticValue(offset,"offset (us)")],[])
            else:
                stat = DiagnosticStatus(2,hostname + ": NTP offset", "Failure to run ntpdate -q", [],[])

            pub.publish(DiagnosticMessage(None, [stat]))
            time.sleep(1)

if __name__ == "__main__":
    ntp_monitor()
