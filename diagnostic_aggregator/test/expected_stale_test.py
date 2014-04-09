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

##\brief Tests that expected items from GenericAnalyzer will appear stale

from __future__ import with_statement
DURATION = 15
PKG = 'diagnostic_aggregator'
import roslib; roslib.load_manifest(PKG)
import rospy, rostest, unittest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from time import sleep
import sys
import threading

def get_raw_name(agg_name):
    return agg_name.split('/')[-1]

class DiagnosticItem:
    def __init__(self, msg):
        self.name = get_raw_name(msg.name)
        self.level = msg.level
        self.message = msg.message

        self.update_time = rospy.get_time()

    def is_stale(self):
        return rospy.get_time() - self.update_time > 5

    def update(self, msg):
        self.level = msg.level
        self.message = msg.message

        self.update_time = rospy.get_time()

class TestExpectedItemsStale(unittest.TestCase):
    def __init__(self, *args):
        super(TestExpectedItemsStale, self).__init__(*args)

        self._mutex = threading.Lock()

        self._expecteds = {}
        self._agg_expecteds = {}

        rospy.init_node('test_expected_stale')        
        self._starttime = rospy.get_time()

        sub = rospy.Subscriber("/diagnostics", DiagnosticArray, self.diag_cb)
        sub_agg = rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.diag_agg_cb)

    def diag_cb(self, msg):
        with self._mutex:
            for stat in msg.status:
                if stat.name.find('expected') == 0:
                    self._expecteds[stat.name] = DiagnosticItem(stat)

    def diag_agg_cb(self, msg):
        with self._mutex:
            for stat in msg.status:
                if stat.name.find('expected') > 0:
                    self._agg_expecteds[get_raw_name(stat.name)] = DiagnosticItem(stat)

    def test_expecteds_arent_stale(self):
        while not rospy.is_shutdown():
            sleep(1.0)
            if rospy.get_time() - self._starttime > DURATION:
                break

        with self._mutex:
            self.assert_(len(self._expecteds) > 0, "No expected items found in raw data!")

            for name, item in self._expecteds.iteritems():
                self.assert_(self._agg_expecteds.has_key(name), "Item %s not found in aggregated diagnostics output" % name)
                if item.is_stale():
                    self.assert_(self._agg_expecteds[name].level == 3, "Stale item in diagnostics, but aggregated didn't report as stale. Item: %s, state: %d" %(name, self._agg_expecteds[name].level))
                else:
                    self.assert_(self._agg_expecteds[name].level == item.level, "Diagnostic level of aggregated, raw item don't match for %s" % name)

if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], TestExpectedItemsStale, sys.argv)
