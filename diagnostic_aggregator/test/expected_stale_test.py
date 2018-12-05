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

##\brief Tests receipt of /diagnostics_agg from diagnostic aggregator

from __future__ import with_statement
PKG = 'diagnostic_aggregator'

TEST_NODE = 'test_expected_stale'
TEST_NAMESPACE = '/my_ns'

#import roslib; roslib.load_manifest(PKG)

import unittest
import rclpy
from rclpy.node import Node
#from rclpy.parameter import Parameter
#import rospy, rostest
from time import sleep
import sys
from optparse import OptionParser
import threading
import types
import time
from diagnostic_msgs.msg import DiagnosticArray
from unittest.mock import Mock


def get_raw_name(agg_name):
    return agg_name.split('/')[-1]

class DiagnosticItem:
    def __init__(self, msg):
        self.name = get_raw_name(msg.name)
        self.level = msg.level
        self.message = msg.message

        self.update_time = time.time()

    def is_stale(self):
        return time.time() - self.update_time > 5

    def update(self, msg):
        self.level = msg.level
        self.message = msg.message

        self.update_time = time.time()


##\brief Uses aggregator parameters to compare diagnostics with aggregated output
class TestAggregator(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
         rclpy.init()
         cls.node = rclpy.create_node(TEST_NODE, namespace=TEST_NAMESPACE)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_expected_stale(self):    
        self._expecteds = {}
        self._agg_expecteds = {} 
        self._mutex = threading.Lock()
        self._starttime = time.time()
        self.Test_pass = False
        self.sub = self.node.create_subscription(DiagnosticArray, '/diagnostics_agg', self.diag_agg_cb)
        self.sub1 = self.node.create_subscription(DiagnosticArray, '/diagnostics', self.diag_cb)
        while self.Test_pass ==False:
            rclpy.spin_once(self.node)

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
            assert(len(self._expecteds) > 0)          
            for name, item in self._expecteds.items():
                if item.is_stale():
                    assert(self._agg_expecteds[name].level == 3) #, "Stale item in diagnostics, but aggregated didn't report as stale. Item: %s, state: %d" %(name, self._agg_expecteds[name].level))
                else:
                    assert(self._agg_expecteds[name].level == item.level) #, "Diagnostic level of aggregated, raw item don't match for %s" % name)
            self.Test_pass =True            
              
                            
if __name__ == '__main__':
   unittest.main()
