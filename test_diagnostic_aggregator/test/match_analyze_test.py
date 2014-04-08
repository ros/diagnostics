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

##\brief Tests that analyzer that matches item will not affect item going into Other

from __future__ import with_statement

DURATION = 10
PKG = 'test_diagnostic_aggregator'
import rospy, rostest, unittest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from time import sleep
import sys
import threading

from optparse import OptionParser

MATCH_NAME = 'Match Item'

def get_raw_name(agg_name):
    return agg_name.split('/')[-1]

def get_header_name(agg_name):
    return '/'.join(agg_name.split('/')[1:-1])

class TestMatchAnalyze(unittest.TestCase):
    def __init__(self, *args):
        super(TestMatchAnalyze, self).__init__(*args)

        parser = OptionParser(usage="usage ./%prog [options]", prog="match_analyze_test.py")
        parser.add_option('--header', action="store", default=None,
                          dest="header", metavar="HEADER",
                          help="Expected header that \"Match Item\" will be under")
        # Option comes with rostest, will fail w/o this line
        parser.add_option('--gtest_output', action="store",
                          dest="gtest")

        options, args = parser.parse_args(rospy.myargv())

        if not options.header:
            parser.error("Option --header is mandatory. Unable to parse args")

        self.header = options.header

        self._mutex = threading.Lock()

        self.match_headers = []

        rospy.init_node('test_match_analyze')
        self._starttime = rospy.get_time()

        sub_agg = rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.diag_agg_cb)
 
    def diag_agg_cb(self, msg):
        with self._mutex:
            for stat in msg.status:
                if stat.name.find(MATCH_NAME) > 0:
                    if self.match_headers.count(get_header_name(stat.name)) == 0:
                        self.match_headers.append(get_header_name(stat.name))

    def test_match_analyze(self):
        while not rospy.is_shutdown():
            sleep(1.0)
            if rospy.get_time() - self._starttime > DURATION:
                break
        
        self.assert_(not rospy.is_shutdown(), "Rospy shutdown!")

        with self._mutex:
            self.assert_(self.header, "Header is none. Option --header not given")
            self.assert_(len(self.match_headers) == 1, "Multiple analyzers reported our item! Headers: %s" % self.match_headers)
            self.assert_(self.match_headers.count(self.header) > 0, "Didn't have item under header \"%s\". Header: \"%s\"" % (self.header, self.match_headers[0]))
         

if __name__ == '__main__':
    if False:
        suite = unittest.TestSuite()
        suite.addTest(TestMatchAnalyze('test_match_analyze'))
        unittest.TextTestRunner(verbosity = 2).run(suite)
    else:
        rostest.run(PKG, sys.argv[0], TestMatchAnalyze, sys.argv)
