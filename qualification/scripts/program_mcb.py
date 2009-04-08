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

from deprecated_srvs.srv import * 

rospy.init_node("mcb_programmer")
rospy.wait_for_service('mcb_conf_results')

result_proxy = rospy.ServiceProxy('mcb_conf_results', StringString)
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

# Use count_mcbs.py to count
# Call script from here, returns 0 if correct count
expected = len(all)
count_cmd = roslib.packages.get_pkg_dir("qualification", True) + "/scripts/count_mcbs.py %s" % expected

retcode = subprocess.call(count_cmd, shell=True)
if retcode != 0:
  print 'Unable to verify MCB count'
  success = False
  sys.exit(255)

for num in all:
  action = StringStringResponse('retry')
  print "Programming MCB %s"%num
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
          success = False
          sys.exit(1)
          break
        
      else: # retcode = 0 -> success
        print "Programmed MCB %s" % num
        action.str = "pass"
  except OSError, e:
    print e
    action = result_proxy("The MCB firmware programing failed to execute. Click YES or NO to exit.:::%s" % details)
    success = False
    sys.exit(2)

if success:
  print "Programming MCB firmware finished."
  action = result_proxy("done")
  sys.exit(0) 

sys.exit(4) 

