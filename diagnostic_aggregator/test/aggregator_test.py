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

PKG = 'diagnostic_aggregator'

import roslib; roslib.load_manifest(PKG)

import unittest
import rospy, rostest
from time import sleep
import sys
from optparse import OptionParser
import threading

from diagnostic_msgs.msg import DiagnosticArray

DURATION = 15
prefix = "/CPP"

def combine_name_prefix(my_prefix, name):
    return '/'.join([prefix, my_prefix, name.replace('/', '')])

def header_name(my_prefix):
    return '/'.join([prefix, my_prefix])

# Supports all options of generic analyzer
def name_to_full_name(name, params):
    for key, value in params.iteritems():
        if not value.has_key('prefix') or not value.has_key('type'):
            return None
        my_prefix = value['prefix']
        if value['type'] != 'GenericAnalyzer':
            return None
        
        if value.has_key('startswith'):
            for sw in value['startswith']:
                if name.startswith(sw):
                    return combine_name_prefix(my_prefix, name), header_name(my_prefix)
        if value.has_key('contains'):
            for con in value['contains']:
                if name.find(con) >= 0:
                    return combine_name_prefix(my_prefix, name), header_name(my_prefix)

        if value.has_key('name'):
            for nm in value['name']:
                if name == nm:
                    return combine_name_prefix(my_prefix, name), header_name(my_prefix)
        if value.has_key('expected'):
            for nm in value['expected']:
                if name == nm:
                    return combine_name_prefix(my_prefix, name), header_name(my_prefix)

    # If we don't have it...
    return combine_name_prefix('Other', name), header_name('Other')

class TestAggregator(unittest.TestCase):
    def __init__(self, *args):
        super(TestAggregator, self).__init__(*args)
        parser = OptionParser(usage="./%prog [options]", prog="aggregator_test.py")
        parser.add_option('--gtest_output', action="store", dest="gtest")
        parser.add_option('--param_name', action="store", dest="param", 
                          default='diag_agg', metavar="PARAM_NAME", 
                          help="Name of parameter that defines analyzers")
        
        self.diag_msgs = {}
        self.agg_msgs = {}
        
        rospy.init_node('test_diag_agg')
        options, args = parser.parse_args(rospy.myargv())
        
        self.params = rospy.get_param(options.param)
        rospy.Subscriber('diagnostics_agg', DiagnosticArray, self.cb)
        rospy.Subscriber('diagnostics', DiagnosticArray, self.diag_cb)
        
        self._mutex = threading.Lock()

    def diag_cb(self, msg):
        self._mutex.acquire()
        for stat in msg.status:
            self.diag_msgs[stat.name] = stat
        self._mutex.release()

    def cb(self, msg):
        self._mutex.acquire()
        for stat in msg.status:
            self.agg_msgs[stat.name] = stat
        self._mutex.release()

    def test_agg(self):
        start = rospy.get_time()
        while not rospy.is_shutdown():
            sleep(1.0)
            if rospy.get_time() - start > DURATION:
                break

        self.assert_(not rospy.is_shutdown(), "Rospy shutdown")

        self._mutex.acquire()

        headers = {}

        # Go through all messages and check that we have them in aggregate
        for name, msg in self.diag_msgs.iteritems():
            agg_name, header = name_to_full_name(name, self.params)
            print 'Name: %s, agg_name: %s', name, agg_name

            self.assert_(agg_name is not None, 'Aggregated name is None for %s' % name)
            self.assert_(self.agg_msgs.has_key(agg_name), 'No matching name found for name: %s, aggregated name: %s' % (name, agg_name))
            self.assert_(msg.level == self.agg_msgs[agg_name].level, 'Status level of original, aggregated messages doesn\'t match. Name: %s, aggregated name: %s.' % (name, agg_name))
            self.assert_(msg.message == self.agg_msgs[agg_name].message, 'Status message of original, aggregated messages doesn\'t match. Name: %s, aggregated name: %s' % (name, agg_name))

            if self.agg_msgs[agg_name].level == 3: # Stale
                self.agg_msgs[agg_name].level = -1
            
            if headers.has_key(header):
                headers[header] = max(headers[header], self.agg_msgs[agg_name].level)
            else:
                headers[header] = self.agg_msgs[agg_name].level

            del self.agg_msgs[agg_name]
            
        # Check that we have headers
        for header, lvl in headers.iteritems():
            if lvl == -1: # All stale
                lvl = 3

            self.assert_(self.agg_msgs.has_key(header), "Header %s not found in messages")
            self.assert_(self.agg_msgs[header].level == lvl, "Level of header %s doesn't match expected.")
            del self.agg_msgs[header]

        # Check that we have the main header message
        self.assert_(len(self.agg_msgs) == 1, "Incorrect number of messages remaining: %d. Messages: %s" % (len(self.agg_msgs), str(self.agg_msgs)))
        
        self.assert_(self.agg_msgs.has_key(prefix), "Global prefix not found in messages: %s. Messages: %s" % (prefix, str(self.agg_msgs)))
                    

        self._mutex.release()


if __name__ == '__main__':
    print 'SYS ARGS:', sys.argv
    rostest.run(PKG, sys.argv[0], TestAggregator, sys.argv)
