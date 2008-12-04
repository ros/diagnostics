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

import rostools
import rostools.packspec
rostools.update_path('qualification')

import rospy
from std_srvs.srv import *
from qualification.srv import *
from robot_srvs.srv import *

rospy.init_node("run_selftest", anonymous=True)

selftestname = rospy.resolve_name('node') + '/self_test'

test_service = rospy.ServiceProxy(selftestname, SelfTest)
result_service = rospy.ServiceProxy('test_result', TestResult)

rospy.wait_for_service(selftestname, 5)
result = test_service.call(SelfTestRequest(),60)

r = TestResultRequest()
if (result.passed):
    r.result = r.RESULT_PASS
else:
    r.result = r.RESULT_FAIL

r.text_result = ""

passfail = 'PASS'
i = 1

statdict = {0: 'OK', 1: 'WARN', 2: 'ERROR'}

for stat in result.status:
    if (stat.level > 1):
        passfail = 'FAIL'
    r.text_result += 'Test %2d) %s\n' % (i, stat.name)
    r.text_result += '  [%s]: %s\n' % (statdict[stat.level], stat.message)
    for val in stat.values:
        r.text_result += '   [%s] = %f\n' % (val.label, val.value)
    i += 1

r.plots = []

rospy.wait_for_service('test_result')
result_service.call(r)
