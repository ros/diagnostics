#!/usr/bin/env python
# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

PKG = 'self_test'

SRV_NAME = 'self_test'

import unittest
import rclpy

import sys
from optparse import OptionParser

from diagnostic_msgs.srv import SelfTest
#from diagnostic_msgs.srv import SelfTest, SelfTestRequest, SelfTestResponse

class TestSelfTest(unittest.TestCase):

    def test_self_test(self):
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
        self.exception = "--exception" 
        rclpy.init(args=args)
        
        node = rclpy.create_node('SelfTest_node_cli')
        cli = node.create_client(SelfTest,SRV_NAME)
        while not cli.wait_for_service(timeout_sec=1.0):
            print('service not available, waiting again...')
        req = SelfTest.Request()
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            #node.get_logger().info('Result of add_two_ints: %s' % future.result())
            res = future.result();
        else:
            node.get_logger().error('Exception while calling service: %r' % future.exception())

        if self.no_id:
            assert str(res.id) == '', 'Result had node ID even though ID was not expected. ID: %s' % res.id
        else:
            assert res.id != '', 'Result had no node ID'

        if self.expect_fail or self.exception:
            assert res.passed == b'\x00', 'Self test passed, but it shouldnt have. Result: %d' % res.passed

            max_val = b'\x00'
            for tst in res.status:
                max_val = max(max_val, tst.level)

            assert max_val > b'\x00', 'self test failed, but no sub tests reported a failure or warning'
        else:
            assert res.passed, 'Self test failed, but we expected a pass'

            for tst in res.status:
                print(tst)
                assert tst.level == b'\x00', 'Self test subtest failed, but we marked it as a pass'
                

        if self.exception:
            found_ex = False
            for tst in res.status:
                if tst.message.find('exception') > -1:
                    found_ex = True

            assert found_ex, 'Self test threw and exception, but we didnt catch it and report it'

    

if __name__ == '__main__':
    unittest.main()
