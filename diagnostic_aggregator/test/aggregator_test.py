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

from diagnostic_msgs.msg import DiagnosticArray

prefix = ""

cb_1 = 0
cb_2 = 0
##\brief Removes name chaff (ex: 'tilt_hokuyo_node: Frequency' to 'Frequency')
def fix_sub_name(name, remove_prefixes):
    last = str(name)
    for start_name in remove_prefixes:
        if last.startswith(start_name):
            last = last[len(start_name):]
        if last.startswith(':'):
            last = last[1:]
        while last.startswith(' '):
            last = last[1:]
    
    return last

def combine_name_prefix(my_prefix, name, remove_prefixes):
    fixed = fix_sub_name(name.replace('/', ''), remove_prefixes)
    return '/'.join([prefix, my_prefix, fixed])

def header_name(my_prefix):
    return '/'.join([prefix, my_prefix])

def _get_params_list(params):
    out = []
    if type(params) in (list, tuple):
        for p in params:
            out.append(str(p))
        return out
    return [ str(params) ]

def name_to_full_generic(name, my_prefix, value, header=False):
    remove_prefixes = []
    if 'remove_prefix' in value:
        remove_prefixes = _get_params_list(value['remove_prefix'])

    if 'find_and_remove_prefix' in value:
        for rp in _get_params_list(value['find_and_remove_prefix']):
            remove_prefixes.extend(_get_params_list(value['find_and_remove_prefix']))
        for sw in _get_params_list(value['find_and_remove_prefix']):
            if name.startswith(sw):
                if header:
                    return header_name(my_prefix)
                return combine_name_prefix(my_prefix, name, remove_prefixes)

    if 'startswith' in value:
        for sw in _get_params_list(value['startswith']):
            if name.startswith(sw):
                if header:
                    print(header_name(my_prefix))
                    return header_name(my_prefix)
                return combine_name_prefix(my_prefix, name, remove_prefixes)

    if 'contains' in value:
        for con in _get_params_list(value['contains']):
            if name.find(con) >= 0:
                if header:
                    return header_name(my_prefix)
                return combine_name_prefix(my_prefix, name, remove_prefixes)

    if 'name' in value:
        for nm in _get_params_list(value['name']):
            if name == nm:
                if header:
                    return header_name(my_prefix)
                return combine_name_prefix(my_prefix, name, remove_prefixes)

    if 'expected' in value:
        for nm in _get_params_list(value['expected']):
            if name == nm:
                if header:
                    return header_name(my_prefix)
                return combine_name_prefix(my_prefix, name, remove_prefixes)


    return None

def name_to_agg_name(name, params):
    for key, value in params.items():
        my_prefix = value['path']
        if value['type'] == 'GenericAnalyzer' or value['type'] == 'diagnostic_aggregator/GenericAnalyzer':
            generic_name = name_to_full_generic(name, my_prefix, value)
            if generic_name is not None:
                return generic_name
        else:
            return None

    # If we don't have it...
    return combine_name_prefix('Other', name, [])

# Returns header name for particular item
def name_to_agg_header(name, params):
    for key, value in params.items():
        my_prefix = value['path']
        if value['type'] == 'GenericAnalyzer' or value['type'] == 'diagnostic_aggregator/GenericAnalyzer':
            generic_name = name_to_full_generic(name, my_prefix, value, header=True)
            if generic_name is not None:
                return generic_name
        else:
            return None

    # If we don't have it...
    return header_name('Other')
    key, sep, value = line.strip().partition(" ")
    return int(key), value


##\brief Uses aggregator parameters to compare diagnostics with aggregated output
class TestAggregator(Node):
    def __init__(self):
        super().__init__('TestAggregator')
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
        
        self.diag_msgs = {}
        self.agg_msgs = {}
        
        self.params = { 'prefix1': { 'type': 'diagnostic_aggregator/GenericAnalyzer', 'path': 'First','remove_prefix': 'prefix1' , 'find_and_remove_prefix': 'find1_items', 'startswith': 'pref1a', 'contains': 'contains1a', 'name': 'name1' } }
        
        global prefix
        global cb_1
        self.create_subscription(DiagnosticArray, 'diagnostics_agg', self.cb)
        self.create_subscription(DiagnosticArray, 'diagnostics', self.diag_cb)
        self._mutex = threading.Lock()

    def diag_cb(self, msg):
        with self._mutex:
            for stat in msg.status:
                self.diag_msgs[stat.name] = stat

    def cb(self, msg):
            with self._mutex:
                for stat in msg.status:
                    self.agg_msgs[stat.name] = stat
                    length = len(self.agg_msgs)

                all_headers = {}
                for name, msg in self.agg_msgs.items():
                    assert(name.startswith('/'))#, "Aggregated name %s doesn't start with \"/\"" % name)

                for name, msg in self.diag_msgs.items():
                    agg_name = name_to_agg_name(name, self.params)
                    assert(agg_name is not None)#, 'Aggregated name is None for %s' % name
                    assert(msg.level == self.agg_msgs[agg_name].level)
                    assert(msg.message == self.agg_msgs[agg_name].message)#, 'Status message of original, aggregated messages doesn\'t match. Name: %s, aggregated name: %s' % (name, agg_name))
                    if self.agg_msgs[agg_name].level == 3: # Stale
                        self.agg_msgs[agg_name].level = -1
                    

                    header = name_to_agg_header(name, self.params)
                    if header in all_headers:
                        all_headers[header] = max(all_headers[header], self.agg_msgs[agg_name].level)
                    else:
                        all_headers[header] = self.agg_msgs[agg_name].level

                    
                    del self.agg_msgs[agg_name]


                # Go through all messages and check that we have them in aggregate
                            # Check that we have all_headers
                for header, lvl in all_headers.items():
                    # If everything is stale, report stale. Otherwise, it should report an error
                    if lvl == -1:
                        lvl = 3

                    if header in self.agg_msgs:#, "Header %s not found in messages" % header)
                        assert(self.agg_msgs[header].level == lvl)#, "Level of header %s doesn't match expected value." % header)
                    del self.agg_msgs[header]

            
               
def main(args=None):
    rclpy.init(args=args)
    node = TestAggregator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()
