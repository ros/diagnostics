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

from diagnostic_msgs.msg import DiagnosticMessage, DiagnosticStatus, DiagnosticValue, DiagnosticString
import sys
import rospy
import socket
from subprocess import Popen, PIPE

import time


NAME = 'disk_usage_monitor'

def disk_usage_monitor(device, offset=5):
    pub = rospy.Publisher("/diagnostics", DiagnosticMessage)
    rospy.init_node(NAME, anonymous=True)

    hostname = socket.gethostbyaddr(socket.gethostname())[0]

    while not rospy.is_shutdown():
        try:
            p = Popen(["df", "-P", "--block-size=1G", device], stdout=PIPE, stdin=PIPE, stderr=PIPE)
            res = p.wait()
            (o,e) = p.communicate()
        except OSError, (errno, msg):
            if errno == 4:
                break #ctrl-c interrupt
            else:
                raise
        if (res == 0):
            second_row = o.split("\n")[1]
            g_available = float(second_row.split()[-3])
            #print second_row, g_available
            #sys.exit(-1)
            
            component_string = "Disk Usage for on host "+ hostname + " for device: " +device
           
            if (g_available > offset):
                stat = DiagnosticStatus(0,component_string, "Acceptable Disk Space Avaiable", [DiagnosticValue(float(g_available),"free space (GB)")],[])
            elif (g_available > 1):
                stat = DiagnosticStatus(1,component_string, "Low Disk Space Avaiable", [DiagnosticValue(float(g_available),"free space (GB)")],[])
            else:
                stat = DiagnosticStatus(2,component_string, "Critically Low Disk Space Avaiable", [DiagnosticValue(float(g_available),"free space (GB)")],[])
        else:
            stat = DiagnosticStatus(2,component_string, "Failure to run \"df -P --block-size\"", [],[])

        pub.publish(DiagnosticMessage(None, [stat]))
        time.sleep(1)

def disk_usage_monitor_main(argv=sys.argv):
    import optparse
    parser = optparse.OptionParser(usage="usage: disk_usage_monitor device [size in GB default = 5]")
    options, args = parser.parse_args(argv[1:])
    if len(args) == 1:
        device, offset = args[0], 5
    elif len(args) == 2:
        device, offset = args
        try:
            offset = int(offset)
        except:
            parser.error("size-tolerance must be a number")
    else:
        parser.error("Invalid arguments")
    disk_usage_monitor(device, offset)

if __name__ == "__main__":
    try:
        disk_usage_monitor_main(rospy.myargv())
    except KeyboardInterrupt: pass
    
