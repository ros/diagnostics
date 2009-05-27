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
import rospy, sys,time
import subprocess
from optparse import OptionParser
from qualification.srv import *
from pr2_power_board.srv import *
import traceback

def main():
  parser = OptionParser()
  parser.add_option("--srv=", type="string", dest="service", action="store", default="prestartup_done")
  parser.add_option("--cmd=", type="string", dest="commands", action="append")

  options, args = parser.parse_args()

  rospy.init_node("power_cmd")
  
  done_proxy = rospy.ServiceProxy(options.service, ScriptDone)
  done = ScriptDoneRequest()
  done.result = ScriptDoneRequest.RESULT_OK
  done.script = 'power_board_commands'
  done.failure_msg = ''

  try:
    rospy.wait_for_service('power_board_control', 5)
  except ROSException, e:
    rospy.logerr('Service wait timed out! %s' % traceback.format_exc())
    # Timeout exceeded, return fail
    done.result = ScriptDoneRequest.RESULT_ERROR
    done.failure_msg = 'Power bower service timed out! %s\n' % traceback.format_exc()
    try:
      rospy.wait_for_service(options.service, 5)
      done_proxy.call(done)
    finally:
      time.sleep(2)

  control_proxy = rospy.ServiceProxy('power_board_control', PowerBoardCommand)
  
  is_first = True

  try:
    for power_cmd in options.commands:
      
      # Wait for previous power cmd to take effect
      if not is_first:
        time.sleep(1)
        
      is_first = False
      for j in range(0, 3):
        # Call power command service
        resp = control_proxy(j, power_cmd, 0)

        rospy.logout('CMD: %s %s. Received return code %d' % 
                     (j, power_cmd, resp.retval))
        if resp.retval != 0:
          details = 'Commanded power board, return code %s.\n\n' % resp.retval
          if resp.retval == -1:
            details += 'Power board may not be connected. Check network connections.'
          
          done.result = ScriptDoneRequest.RESULT_ERROR
          done.failure_msg = details
          break

  except Exception, e:
    rospy.logerr('Caught exception!')
    rospy.logerr(traceback.format_exc())
    done.result = ScriptDoneRequest.RESULT_ERROR
    done.failure_msg = 'Caught exception.\n%s' % traceback.format_exc()

  try:
    # If we're all done...
    rospy.wait_for_service(options.service, 5)
    done_proxy.call(done)
  finally:
    pass

if __name__ == '__main__':
  main()

 
