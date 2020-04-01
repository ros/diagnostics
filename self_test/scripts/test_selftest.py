#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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
##\brief Tests for valid self test calls

PKG = 'self_test'
import roslib; roslib.load_manifest(PKG)

SRV_NAME = 'my_node/self_test'

import unittest
import rospy, rostest

import sys
from optparse import OptionParser

from diagnostic_msgs.srv import SelfTest, SelfTestRequest, SelfTestResponse

class TestSelfTest(unittest.TestCase):
    def __init__(self, *args):
        super(TestSelfTest, self).__init__(*args)

        parser = OptionParser(usage="usage ./%prog [options]", prog="test_selftest.py")
        parser.add_option('--no-id', action="store_true",
                          dest="no_id", default=False,
                          help="No ID expected from self test")
        parser.add_option('--expect-fail', action="store_true",
                          dest="expect_fail", default=False,
                          help="Self test should fail")
        parser.add_option('--exception', action="store_true",
                          dest="exception", default=False,
                          help="Self test should throw exception and we should get error message")
        # Option comes with rostest, will fail w/o this line
        parser.add_option('--gtest_output', action="store",
                          dest="gtest")

        options, args = parser.parse_args()

        self.no_id = options.no_id
        self.expect_fail = options.expect_fail
        self.exception = options.exception
        

    def test_self_test(self):
        proxy = rospy.ServiceProxy(SRV_NAME, SelfTest)

        try:
            rospy.wait_for_service(SRV_NAME, 15)
        except Exception as e:
            self.assert_(False, "Service %s did not respond. Unable to test self_test" % SRV_NAME)

        try:
            res = proxy()
        except Exception as e:
            import traceback
            self.assert_(False, "Error calling self_test service. Exception: %s" % traceback.format_exc())

        if self.no_id:
            self.assert_(res.id == '', "Result had node ID even though ID was not expected. ID: %s" % res.id)
        else:
            self.assert_(res.id != '', "Result had no node ID")

        if self.expect_fail or self.exception:
            self.assert_(res.passed == 0, "Self test passed, but it shouldn't have. Result: %d" % res.passed)

            max_val = 0
            for tst in res.status:
                max_val = max(max_val, tst.level)

            self.assert_(max_val > 0, "Self test failed, but no sub tests reported a failure or warning")
        else:
            self.assert_(res.passed, "Self test failed, but we expected a pass")

            for tst in res.status:
                self.assert_(tst.level == 0, "Self test subtest failed, but we marked it as a pass")
                

        if self.exception:
            found_ex = False
            for tst in res.status:
                if tst.message.find('exception') > -1:
                    found_ex = True

            self.assert_(found_ex, "Self test threw and exception, but we didn't catch it and report it")

            
if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], TestSelfTest, sys.argv)
