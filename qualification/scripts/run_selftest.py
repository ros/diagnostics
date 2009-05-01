#!/usr/bin/env python
#
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

import roslib
roslib.load_manifest('qualification')

import rospy
from qualification.srv import *
from robot_srvs.srv import *

import sys
from time import sleep

rospy.init_node("run_selftest", anonymous=True)

node_name = 'node'
node_id = 'NONE'

try:
    node_name = rospy.resolve_name('node_name')
    selftestname = node_name + '/self_test'
    rospy.logout('Testing %s' % selftestname)
    
    test_service = rospy.ServiceProxy(selftestname, SelfTest)
    result_service = rospy.ServiceProxy('test_result', TestResult)
    
    rospy.wait_for_service(selftestname)
    sleep(5)


    result = test_service.call(SelfTestRequest(), 90)
    rospy.logout('Received self test service.')
    
    r = TestResultRequest()
    if (result.passed):
        r.result = r.RESULT_PASS
    else:
        r.result = r.RESULT_HUMAN_REQUIRED #RESULT_FAIL

    passfail = 'PASS'
    i = 1
    
    if result.id is not None and result.id != '':
        node_id = result.id

    html = "<p><b>Item ID: %s, using node name %s.</b></p>\n" % (node_id, node_name)

    statdict = {0: 'OK', 1: 'WARN', 2: 'ERROR'}
    
    for stat in result.status:
        if (stat.level > 1):
            passfail = 'FAIL'
        html += "<p><b>Test %2d) %s</b>\n" % (i, stat.name)
        html +=  '<br>Result %s: %s</p>\n' % (statdict[stat.level], stat.message)
        
        if len(stat.values) > 0:
            html += "Diagnostic Values<br>\n"
            html += "<table border=\"1\" cellpadding=\"2\" cellspacing=\"0\">\n"
            html += "<tr><b><td>Label</td><td>Value</td></b></tr>\n"
            for val in stat.values:
                html += "<tr><td>%s</td><td>%f</td></tr>\n" % (val.label, val.value)
                html += "</table>\n"
        
        if len(stat.strings) > 0:
            html += "Diagnostic Strings<br>\n"
            html += "<table border=\"1\" cellpadding=\"2\" cellspacing=\"0\">\n"
            html += "<tr><b><td>Label</td><td>Value</td></b></tr>\n"
            for val in stat.values:
                html += "<tr><td>%s</td><td>%s</td></tr>\n" % (val.label, val.value)
                html += "</table>\n"

        html += "<hr size=\"2\">\n"
        
        i += 1

    r.plots = []
    r.html_result = html
    r.text_summary = 'Node ID: %s, node name %s. Self test result: %s' % (node_id, node_name, passfail)
    
    rospy.wait_for_service('test_result')
    result_service.call(r)

except Exception, e:
    msg = 'Caught exception testing device id %s, node name %s.</p>\n' % (node_id, node_name)
    rospy.logerr(msg)
    rospy.logerr(str(e))
    rospy.wait_for_service('test_result', 10)
    r = TestResultRequest()
    r.plots = []
    r.html_result = '<p>%s</p><p><b>Exception:</b><br>%s</p>' % (msg, str(e))
    r.text_summary = msg
    r.result = r.RESULT_FAIL
    result_service.call(r)
    sys.exit(255)

sys.exit(0)


