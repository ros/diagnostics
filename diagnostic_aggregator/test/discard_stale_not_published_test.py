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

##\author Guglielmo Gemignani

##\brief Tests that expected items from GenericAnalyzer will be removed after the timeout if discard_stale is set to true

DURATION = 10
PKG = 'diagnostic_aggregator'

import roslib; roslib.load_manifest(PKG)
import rospy, rostest, unittest
from diagnostic_msgs.msg import DiagnosticArray
from time import sleep
import sys
import threading


class TestDiscardStale(unittest.TestCase):
    def __init__(self, *args):
        super(TestDiscardStale, self).__init__(*args)

        self._mutex = threading.Lock()

        self._agg_expecteds = []
        
        rospy.init_node('test_expected_stale')        
        sub_agg = rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.diag_agg_cb)
        self._start_time = rospy.Time.now()

    def diag_agg_cb(self, msg):
        with self._mutex:
            self._agg_expecteds = []
            for stat in msg.status:
                self._agg_expecteds.append(stat)

    def test_discard_stale(self):
        while not rospy.is_shutdown():
            sleep(1.0)
            if rospy.Time.now() - self._start_time > rospy.Duration(DURATION):
                break

        with self._mutex:
            self.assert_(len(self._agg_expecteds) == 1,
                         "There should only be one expected aggregated item left, {} found instead!".
                         format(len(self._agg_expecteds)))
            self.assert_(self._agg_expecteds[0].name == "/Nonexistent1",
                         "The name of the only aggregate message should be '/Nonexistent1'!")


if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], TestDiscardStale, sys.argv)
