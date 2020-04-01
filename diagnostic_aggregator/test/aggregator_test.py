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

import roslib; roslib.load_manifest(PKG)

import unittest
import rospy, rostest
from time import sleep
import sys
from optparse import OptionParser
import threading
import types

from diagnostic_msgs.msg import DiagnosticArray

prefix = ""

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
        if not 'path' in value or not 'type' in value:
            return None
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
        if not 'path' in value or not 'type' in value:
            return None
        my_prefix = value['path']
        if value['type'] == 'GenericAnalyzer' or value['type'] == 'diagnostic_aggregator/GenericAnalyzer':
            generic_name = name_to_full_generic(name, my_prefix, value, header=True)
            if generic_name is not None:
                return generic_name
        else:
            return None

    # If we don't have it...
    return header_name('Other')

##\brief Uses aggregator parameters to compare diagnostics with aggregated output
class TestAggregator(unittest.TestCase):
    def __init__(self, *args):
        super(TestAggregator, self).__init__(*args)
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
        
        rospy.init_node('test_diag_agg')
        options, args = parser.parse_args(rospy.myargv())

        global prefix
        prefix = options.base_path
        
        self.params = rospy.get_param(options.param)
        self.duration = options.duration
        rospy.Subscriber('diagnostics_agg', DiagnosticArray, self.cb)
        rospy.Subscriber('diagnostics', DiagnosticArray, self.diag_cb)
        
        self._mutex = threading.Lock()

    def diag_cb(self, msg):
        with self._mutex:
            for stat in msg.status:
                self.diag_msgs[stat.name] = stat


    def cb(self, msg):
        with self._mutex:
            for stat in msg.status:
                self.agg_msgs[stat.name] = stat

    def test_agg(self):
        start = rospy.get_time()
        while not rospy.is_shutdown():
            sleep(1.0)
            if rospy.get_time() - start > self.duration:
                break

        self.assert_(not rospy.is_shutdown(), "Rospy shutdown")

        with self._mutex:
            all_headers = {}

            for name, msg in self.agg_msgs.items():
                self.assert_(name.startswith('/'), "Aggregated name %s doesn't start with \"/\"" % name)

            # Go through all messages and check that we have them in aggregate
            for name, msg in self.diag_msgs.items():
                agg_name = name_to_agg_name(name, self.params)
                
                self.assert_(agg_name is not None, 'Aggregated name is None for %s' % name)
                self.assert_(agg_name in self.agg_msgs, 'No matching name found for name: %s, aggregated name: %s' % (name, agg_name))
                self.assert_(msg.level == self.agg_msgs[agg_name].level, 'Status level of original, aggregated messages doesn\'t match. Name: %s, aggregated name: %s.' % (name, agg_name))
                self.assert_(msg.message == self.agg_msgs[agg_name].message, 'Status message of original, aggregated messages doesn\'t match. Name: %s, aggregated name: %s' % (name, agg_name))
                
                # This is because the analyzers only reports stale if
                # all messages underneath it are stale
                if self.agg_msgs[agg_name].level == 3: # Stale
                    self.agg_msgs[agg_name].level = -1
            
                header = name_to_agg_header(name, self.params)
                if header in all_headers:
                    all_headers[header] = max(all_headers[header], self.agg_msgs[agg_name].level)
                else:
                    all_headers[header] = self.agg_msgs[agg_name].level

                del self.agg_msgs[agg_name]
            
            # Check that we have all_headers
            for header, lvl in all_headers.items():
                # If everything is stale, report stale. Otherwise, it should report an error
                if lvl == -1:
                    lvl = 3

                self.assert_(header in self.agg_msgs, "Header %s not found in messages" % header)
                self.assert_(self.agg_msgs[header].level == lvl, "Level of header %s doesn't match expected value." % header)
                del self.agg_msgs[header]

        # Check that we have the main header message
            if len(prefix) > 0:
                self.assert_(len(self.agg_msgs) == 1, "Incorrect number of messages remaining: %d. Messages: %s" % (len(self.agg_msgs), str(self.agg_msgs)))
                
                self.assert_(prefix in self.agg_msgs, "Global prefix not found in messages: %s. Messages: %s" % (prefix, str(self.agg_msgs)))
            else:
                self.assert_(len(self.agg_msgs) == 0, "Incorrect number of messages remaining: %d. Messages: %s. Expected 0." % (len(self.agg_msgs), str(self.agg_msgs)))
                


if __name__ == '__main__':
    print('SYS ARGS:', sys.argv)
    rostest.run(PKG, sys.argv[0], TestAggregator, sys.argv)
