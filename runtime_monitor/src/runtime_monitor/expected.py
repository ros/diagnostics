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
#

#import roslib
#roslib.load_manifest(PKG)

import rospy
from diagnostic_msgs.msg import *

stat_dict = { 0: 'OK', 1: 'Warning', 2: 'Error' }

def test(latest_msgs, parameters, test_name):
    status = DiagnosticStatus()
    status.name = 'Expected %s' % test_name
    status.level = 0
    status.message = 'OK'
    status.strings = []
    status.values = []

    if "expected_present" in parameters:
        for name in parameters["expected_present"]:
            if name in latest_msgs and rospy.get_time() - latest_msgs[name]["last_time"] < 3.0:
                msg = 'OK'
            elif name in latest_msgs:
                msg = 'Stale - Error'
                status.level = max(status.level, 2)
            else:
                msg = 'Missing - Error'
                status.level = max(status.level, 2)
            status.strings.append(DiagnosticString(label = name, value = msg))


    if "desired_present" in parameters:
        for name in parameters["desired_present"]:
            if name in latest_msgs and rospy.get_time() - latest_msgs[name]["last_time"] < 3.0:
                msg = 'OK'
            elif name in latest_msgs:
                msg = 'Stale - Warning'
                status.level = max(status.level, 1)
            else:
                msg = 'Missing - Warning'
                status.level = max(status.level, 1)
            status.strings.append(DiagnosticString(label = name, value = msg))

    status.message = stat_dict[status.level]

    return status

    
