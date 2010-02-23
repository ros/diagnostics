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

##\brief Tests that two analyzers can match and analyze a single item

from __future__ import with_statement
DURATION = 10
PKG = 'diagnostic_aggregator'
import roslib; roslib.load_manifest(PKG)
import rospy, rostest, unittest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from time import sleep
import sys
import threading

MULTI_NAME = 'multi'
HEADER1 = 'Header1'
HEADER2 = 'Header2'

def get_raw_name(agg_name):
    return agg_name.split('/')[-1]

def get_header_name(agg_name):
    return '/'.join(agg_name.split('/')[1:-1])

class DiagnosticItem:
    def __init__(self, msg):
        self.name = get_raw_name(msg.name)
        self.header = get_header_name(msg.name)
        self.level = msg.level
        self.message = msg.message

        self.update_time = rospy.get_time()

    def is_stale(self):
        return rospy.get_time() - self.update_time > 5

    def update(self, msg):
        self.level = msg.level
        self.message = msg.message

        self.update_time = rospy.get_time()

class TestMultipleMatch(unittest.TestCase):
    def __init__(self, *args):
        super(TestMultipleMatch, self).__init__(*args)

        self._mutex = threading.Lock()

        self._multi_items = {}

        rospy.init_node('test_multiple_match')
        self._starttime = rospy.get_time()

        sub_agg = rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.diag_agg_cb)
 
    def diag_agg_cb(self, msg):
        with self._mutex:
            for stat in msg.status:
                if stat.name.find(MULTI_NAME) > 0:
                    self._multi_items[get_header_name(stat.name)] = DiagnosticItem(stat)

    def test_multiple_match(self):
        while not rospy.is_shutdown():
            sleep(1.0)
            if rospy.get_time() - self._starttime > DURATION:
                break
        
        self.assert_(not rospy.is_shutdown(), "Rospy shutdown!")

        with self._mutex:
            self.assert_(self._multi_items.has_key(HEADER1), "Didn't have item under %s. Items: %s" % (HEADER1, self._multi_items))
            self.assert_(self._multi_items[HEADER1].name == MULTI_NAME, "Item name under %s didn't match %s" % (HEADER1, MULTI_NAME))

            self.assert_(self._multi_items.has_key(HEADER2), "Didn't have item under %s" % HEADER2)
            self.assert_(self._multi_items[HEADER2].name == MULTI_NAME, "Item name under %s didn't match %s" % (HEADER2, MULTI_NAME))
         

if __name__ == '__main__':
    if False:
        suite = unittest.TestSuite()
        suite.addTest(TestMultipleMatch('test_multiple_match'))
        unittest.TextTestRunner(verbosity = 2).run(suite)
    else:
        rostest.run(PKG, sys.argv[0], TestMultipleMatch, sys.argv)
