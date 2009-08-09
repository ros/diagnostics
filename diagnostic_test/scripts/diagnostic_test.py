#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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

## A basic node to monitor diagnostics for expected status

PKG = 'diagnostic_test'

import roslib; roslib.load_manifest(PKG)

import sys, time
import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue, DiagnosticString

NAME = 'diagnostic_test'

latest_messages = {}
test_name = 'uninitialized'
package = 'uninitialized'
last_runtime = 0
startup_delay = 5.0
start_time = 0

## Records status as dictionary by labels for strings, values
## Also stores last time of message, allows stale checks.
def status_to_map(status):
    str_map = {}
    for val in status.values:
        str_map[val.key] = val.value;
    str_map["name"]= status.name
    str_map["message"] = status.message
    str_map["level"] = status.level

    # Store last time message was recorded
    str_map["last_time"] = rospy.get_time()
    return str_map
    
## The test takes the latest_messages dictionary, the private parameters
## and the status name (the name of the DiagnosticStatus message
## it publishes)
def analyze(test_impl, params):
    return test_impl.test(latest_messages, params)

def callback(message, args):
    for s in message.status:
        latest_messages[s.name] = status_to_map(s)
    execute_test(args)

## Performs designated test using latest messages. Publishes to /diagnostics
## Never tests greater than max frequency.
##@param args (Test implementation, private parameters for test)
def execute_test(args):
    global publisher
    global last_runtime

    if rospy.get_time() < start_time + startup_delay:
        rospy.logdebug("Waiting to for startup delay")
        return
    
    # Don't execute at greater than max frequency
    time_step = rospy.get_time() - last_runtime
    if 1.0 / time_step > options.max_freq:
        return
    else:
        last_runtime = rospy.get_time()
    
    test_impl, params = args

    msg = DiagnosticArray()
    msg.status = [analyze(test_impl, params)]
    publisher.publish(msg)

## Starts up test, subscribes and publishes to diagnostics
def diagnostic_test(package, test_name):
    # retrieve the test implementation
    roslib.load_manifest(package)
    __import__("%s.%s"%(package, test_name))
    try:
        pypkg = sys.modules[package]
    except KeyError:
        print >> sys.stderr, "ERROR: cannot locate test package %s"%package
        rospy.logerr("cannot locate test package %s"%package)
        sys.exit(1)
    test_impl = getattr(pypkg, test_name)    

    # must be inited before reading parameters
    rospy.init_node(NAME, anonymous=True)
    global start_time, last_runtime
    last_runtime  = rospy.get_time()
    start_time = rospy.get_time()

    # get it's parameters
    params = rospy.get_param("~")

    rospy.Subscriber("/diagnostics", DiagnosticArray, callback, (test_impl, params))

    # Publish results in diagnostics
    global publisher
    publisher = rospy.Publisher('/diagnostics', DiagnosticArray)
    
    # Always executes at greater than the min frequency
    while not rospy.is_shutdown():
        if rospy.get_time() - last_runtime > 1/options.min_freq:
            execute_test((test_impl, params))
        time.sleep(0.5/options.min_freq)
        
if __name__ == '__main__': 
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog [options]", prog='diagnostic_test.py')
    parser.add_option("--test", metavar="TEST_NAME",
                      dest="test_name", default='', 
                      type="string", help="test name")
    parser.add_option("--package", metavar="ROS_PACKAGE",
                      dest="package", default='diagnostic_test',
                      type="string", help="package test is in")
    parser.add_option("--min_freq", metavar="MIN_FREQ",
                      dest="min_freq", default='0.5', 
                      type="float", help="Minimum Execution Frequency(Hz)")
    parser.add_option("--max_freq", metavar="MAX_FREQ",
                      dest="max_freq", default='1.0', 
                      type="float", help="Maximum Execution Frequency(Hz)")
    parser.add_option("--startup_delay", metavar="STARTUP_DELAY",
                      dest="startup_delay", default='10.0', 
                      type="float", help="Time to wait before Polling(Seconds)")
    

    options, args = parser.parse_args()

    
    # expected or default
    package = options.package 
    startup_delay = options.startup_delay
    
    if options.test_name:
        test_name = options.test_name 
    else:
        parser.error("Diagnostic test must be given test to run")

    try:
        diagnostic_test(package, options.test_name)
    except KeyboardInterrupt, e:
        pass
    print "exiting"
