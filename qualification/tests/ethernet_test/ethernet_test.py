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

import roslib
roslib.load_manifest('qualification')

from robot_msgs.msg import *
from robot_srvs.srv import *
from qualification.msg import *
from qualification.srv import *

import rospy 

NAME = 'ethernet'

import os
import sys
from StringIO import StringIO

def ethernet_test():
    rospy.init_node(NAME)
    
    name = sys.argv[1]
    ip = sys.argv[2]
    
    res = os.popen('ping -f -q -w 1 -s 32768 %s' % (ip)).readlines()

    r = TestResultRequest()
    r.plots = []
    
    s = StringIO()

    if (len(res) > 1):
        tran = float(res[3].split()[0])
        recv = float(res[3].split()[3])

        print >> s, 'Transmitted: %f'%(tran)
        print >> s, 'Received: %f'%(recv)

        if ((tran - recv) <= 2):
          res = os.popen('netperf -H %s -t UDP_STREAM -l 1' % (ip)).readlines()

          speed = float(res[6].split()[3])
          speed_str = ''
          
          r.result = TestResultRequest.RESULT_PASS
          
          if (speed > 900):
            speed_str = 'Gigabit'
          elif (speed > 90):
            speed_str = '100 Megabit'
          else:
            speed_str = '< 10 Megabit'
          
          r.text_summary = speed_str
          r.html_result = '<p>Speed: %f (%s).</p>'%(speed, speed_str)
        else:
          r.text_summary = 'Too many packets lost.'
          r.html_result = '<p>Too many packets lost.</p>'
    else:
      r.text_summary = 'Running ping failed.'
      r.html_result = '<p>Running ping failed!</p>'
        
    p.text = s.getvalue()
    
    # block until the test_result service is available
    rospy.wait_for_service('test_result')
    result_service = rospy.ServiceProxy('test_result', TestResult)
    result_service.call(r)

if __name__ == "__main__":
    ethernet_test()
