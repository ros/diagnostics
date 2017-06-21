#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, TNO IVS, Helmond, Netherlands
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
#  * Neither the name of the TNO IVS nor the names of its
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


import unittest
import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticArray


class TestCPUMonitor(unittest.TestCase):

    def diagnostics_callback(self, msg):
        self._msg = msg

    def test_cpu_monitor_diagnostics(self):
        rospy.init_node('test_cpu_monitor')
        self._expected_level = None
        if rospy.has_param('~expected_level'):
            self._expected_level = rospy.get_param('~expected_level')

        self._subscriber = rospy.Subscriber('diagnostics', DiagnosticArray, self.diagnostics_callback)
        self._msg = None

        while not self._msg:
            rospy.sleep(.1)
        self._subscriber.unregister()

        self.assertEqual(len(self._msg.status), 1)
        status = self._msg.status[0]

        for v in status.values:
            percentage = float(v.value)
            self.assertGreaterEqual(percentage, 0)
            self.assertLessEqual(percentage, 100)

        if self._expected_level:
            self.assertEqual(self._expected_level, status.level)


PKG = 'diagnostics_common_diagnostics'
NAME = 'test_cpu_monitor'
if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestCPUMonitor)
