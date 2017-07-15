#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

##\author Kevin Watts

##\brief Publishes diagnostic messages for diagnostic aggregator unit test

PKG = 'diagnostic_aggregator'

import roslib; roslib.load_manifest(PKG)


import rospy
from time import sleep

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

if __name__ == '__main__':
    rospy.init_node('diag_pub')
    pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
    
    array = DiagnosticArray()
    array.status = [
        # GenericAnalyzer prefix1
        DiagnosticStatus(0, 'pref1a', 'OK', '', []),
        DiagnosticStatus(1, 'pref1b', 'Warning', '', []),
        DiagnosticStatus(0, 'contains1a', 'OK', '', []),
        DiagnosticStatus(0, 'prefix1: contains1b', 'OK', '', []),
        DiagnosticStatus(0, 'name1', 'OK', '', []),
        DiagnosticStatus(0, 'prefix1: expected1a', 'OK', '', []),
        DiagnosticStatus(0, 'prefix1: expected1b', 'OK', '', []),
        DiagnosticStatus(0, 'prefix1: expected1c', 'OK', '', []),
        DiagnosticStatus(0, 'prefix1: expected1d', 'OK', '', []),
        DiagnosticStatus(0, 'find1_items: find_remove1a', 'OK', '', []),
        DiagnosticStatus(0, 'find1_items: find_remove1b', 'OK', '', []),

        # GenericAnalyzer prefix2
        DiagnosticStatus(0, 'contain2a', 'OK', '', []),
        DiagnosticStatus(0, 'contain2b', 'OK', '', []),
        DiagnosticStatus(0, 'name2', 'OK', '', []),

        # OtherAnalyzer for Other
        DiagnosticStatus(2, 'other1', 'Error', '', []),
        DiagnosticStatus(0, 'other2', 'OK', '', []),
        DiagnosticStatus(0, 'other3', 'OK', '', [])]
    array.header.stamp = rospy.get_rostime()

    while not rospy.is_shutdown():
        pub.publish(array)
        sleep(1)
