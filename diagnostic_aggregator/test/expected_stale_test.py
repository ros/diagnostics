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
DURATION = 15

go_1 = 0
go_2 = 0
prefix = ""

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
class TestAggregator(Node):
    def __init__(self):
        super().__init__('test_aggregator')
        parser = OptionParser(usage="./%prog [options]", prog="aggregator_test.py")
        parser.add_option('--gtest_output', action="store", dest="gtest")
        parser.add_option('--param_name', action="store", dest="param", 
                          default='diag_agg', metavar="PARAM_NAME", 
                          help="Name of parameter that defines analyzers")
        parser.add_option('--duration', action="store", dest="duration",
                          default=10, metavar="DURATIION",
                          help="Duration of test")
        parser.add_option('--base_path', action="store", dest="base_path",
                          default="", metavar="BASE_PATH",
                          help="Base path for all output topics")
        self._expecteds = {}
        self._agg_expecteds = {} 
        self._mutex = threading.Lock()
        self._starttime = time.time()
        global prefix
        
        print("Hellow going create subcription")
        self.sub = self.create_subscription(DiagnosticArray, 'diagnostics_agg', self.diag_agg_cb)
        self.sub1 = self.create_subscription(DiagnosticArray, 'diagnostics', self.diag_cb)


    def diag_cb(self, msg):
        print("Hellow:",msg)
        with self._mutex:
            for stat in msg.status:
                if stat.name.find('expected') == 0:
                    self._expecteds[stat.name] = DiagnosticItem(stat)
                    print("expected is :",stat.name)
        
            #go_1=1
            #if go_2 == 1:
            #    self.test_agg();

    def diag_agg_cb(self, msg):
        with self._mutex:
            for stat in msg.status:
                print("diag_agg_cb stat : ",stat)
                if stat.name.find('expected') > 0:
                    self._agg_expecteds[get_raw_name(stat.name)] = DiagnosticItem(stat)
                    print("expected item found in diag_agg_cb:",stat.name)
            print("self._expecteds is ", self._expecteds)       
            assert(len(self._expecteds) > 0)          
            for name, item in self._expecteds.items():
                if item.is_stale():
                    print("================ tes pass ===================")
                    assert(self._agg_expecteds[name].level == 3) #, "Stale item in diagnostics, but aggregated didn't report as stale. Item: %s, state: %d" %(name, self._agg_expecteds[name].level))
                   # print("Stale item in diagnostics, but aggregated didn't report as stale. Item: %s, state: %d" %(name, self._agg_expecteds[name].level))
                else:
                    print("Diagnostic level of aggregated, raw item don't match for:",self._agg_expecteds[name].level ,item.level )
                    assert(self._agg_expecteds[name].level == item.level) #, "Diagnostic level of aggregated, raw item don't match for %s" % name)
                        
           # go_2=1
           # if go_1 == 1:
           #     self.test_agg();
              

    def test_agg(self):
        start = time.time()
        while 1:
            sleep(1.0)
            if time.time() - start > DURATION:
               break

        print("Test cases start ")    
        #self.assert_(not rospy.is_shutdown(), "Rospy shutdown")
        with self._mutex:
            assert(len(self._expecteds) > 0) #, "No expected items found in raw data!")

            for name, item in self._expecteds.items():
                assert(self._agg_expecteds.has_key(name)) #, "Item %s not found in aggregated diagnostics output" % name)
                if item.is_stale():
                    assert(self._agg_expecteds[name].level == 3) #, "Stale item in diagnostics, but aggregated didn't report as stale. Item: %s, state: %d" %(name, self._agg_expecteds[name].level))
                else:
                    assert(self._agg_expecteds[name].level == item.level) #, "Diagnostic level of aggregated, raw item don't match for %s" % name)
        print("Test cases pass ")    
                        
def main(args=None):
    rclpy.init(args=args)
    node = TestAggregator()
 #   node.test_agg()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   # print 'SYS ARGS:', sys.argv
   # rostest.run(PKG, sys.argv[0], TestAggregator, sys.argv)
   main()
