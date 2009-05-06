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

# Author: Kevin Watts

import roslib
roslib.load_manifest('qualification')
import rospy, sys
import subprocess
from optparse import OptionParser
from time import sleep

from deprecated_srvs.srv import * 
from qualification.srv import *

rospy.init_node("mcb_programmer")
rospy.wait_for_service('mcb_conf_results')

result_proxy = rospy.ServiceProxy('mcb_conf_results', StringString)
done_proxy = rospy.ServiceProxy('prestartup_done', ScriptDone)
done = ScriptDoneRequest()
done.script = 'program mcbs'
done.failure_msg = ''
done.result = ScriptDoneRequest.RESULT_OK

parser = OptionParser()
parser.add_option("--wg005=", type="string", dest="wg005", action="append")
parser.add_option("--wg006=", type="string", dest="wg006", action="append")

options, args = parser.parse_args()

path = roslib.packages.get_pkg_dir("qualification", True) + "/fwprog"

success = True

all = []
if options.wg005:
  if options.wg006:
    all = options.wg005 + options.wg006
  else:
    all = options.wg005
else:
  all = options.wg006

count_proxy = rospy.ServiceProxy('count_mcbs', CountBoards)
count_req = CountBoardsRequest()
count_req.expected = len(all)

try:
  #rospy.logout('Counting MCB\'s')
  rospy.wait_for_service('count_mcbs', 15)
  count_resp = count_proxy.call(count_req)

  #rospy.logerr('Count resp: %s' % count_resp.ok)
  if count_resp.ok != CountBoardsResponse.RESULT_OK:
    if count_resp.ok == CountBoardsResponse.RESULT_ERROR:
      rospy.logerr('Count resp: Error!')
      done.result = ScriptDoneRequest.RESULT_ERROR
      rospy.logerr('Logged error')
    else:
      rospy.logerr('Count resp: Error!')
      done.result = ScriptDoneRequest.RESULT_FAIL
      rospy.logerr('Logged error')

    done.failure_msg = 'Unable to count MCB\'s, count_mcbs.py returned with %d, found %d board.' % (count_resp.ok, count_resp.count)
    rospy.logerr(done.failure_msg)
  else:
    rospy.logout('MCB count: ok')
except Exception, e:
  rospy.logerr('Exception counting: %s' % str(e))
  rospy.logerr(e)
  done.result = ScriptDoneRequest.RESULT_ERROR
  done.failure_msg = 'MCB programming failed, caught exception. %s' % str(e)
  success = False


if done.result != ScriptDoneRequest.RESULT_OK:
  try:
    rospy.wait_for_service('prestartup_done')
    done_proxy.call(done)
  except Exception, e:
    rospy.logerr('Caught exception after count ended')
    rospy.logerr(e)
  
for num in all:
  if not success:
    break

  action = StringStringResponse('retry')
  #print "Programming MCB %s"%num
  details = ''
  try:
    while(action.str == "retry"):
      filename = path + "/*.bit" 
      cmd = "LD_LIBRARY_PATH=" + path + " " + path + "/fwprog" + " -i eth0 -p %s %s"%(num, filename)
      
      p = subprocess.Popen(cmd, stdout = subprocess.PIPE, stderr = subprocess.PIPE, shell=True)
      stdout, stderr = p.communicate()
      retcode = p.returncode
      
      details = 'Ran fwprog on MCB %s, returned %s.\n\n' % (num, retcode)
      details += 'CMD:\n' + cmd + '\n'
      details += 'STDOUT:\n' + stdout
      if len(stderr) > 5:
        details += '\nSTDERR:\n' + stderr
        
      if retcode != 0:
        action = result_proxy("Programming MCB firmware failed for MCB #%s with error %d! Would you like to retry?:::%s"%(num, retcode, details))

        if action.str == "fail":
          print "Programming MCB firmware failed for %s!"%num
          done = ScriptDoneRequest()
          done.result = ScriptDoneRequest.RESULT_FAIL
          done.script = 'Program MCB'
          done.failure_msg = 'MCB programming failed, user chose not to retry!'
          success = False
          break
        
      else: # retcode = 0 -> success
        #print "Programmed MCB %s" % num
        action.str = "pass"
  except Exception, e:
    rospy.logerr('Caught exception attempting to program MCB\'s')
    rospy.logerr(str(e))
    done = ScriptDoneRequest()
    done.result = ScriptDoneRequest.RESULT_ERROR
    done.script = 'Program MCB'
    done.failure_msg = 'MCB programming failed, caught exception. %s' % str(e)
    success = False
    break
 

if success:
  rospy.logout("Programming MCB firmware finished.")
  done.result = ScriptDoneRequest.RESULT_OK

  
try:
  action = result_proxy("done")
  rospy.wait_for_service('prestartup_done', 5)
  done_proxy.call(done)
  rospy.spin()
except Exception, e:
  rospy.logerr(e)
  rospy.spin()


