#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, Magazino GmbH.
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
#  * Neither the name of Magazino GmbH nor the names of its
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

# \author Guglielmo Gemignani

# \brief Publishes messages for aggregator testing of discard stale flag

from time import sleep

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import rospy


if __name__ == '__main__':
    rospy.init_node('diag_pub')
    pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)

    start_time = rospy.get_time()

    while not rospy.is_shutdown():
        array = DiagnosticArray()
        array.header.stamp = rospy.get_rostime()

        # after 2 seconds we will publish only an empty diagnostics
        if rospy.get_time() - start_time < 3:
            array.status = [DiagnosticStatus(1, 'nonexistent2', 'WARN', '', [])]

        pub.publish(array)
        sleep(1.0)
