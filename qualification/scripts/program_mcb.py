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
import rospy, sys
import subprocess
from optparse import OptionParser

from deprecated_srvs.srv import * 

rospy.init_node("mcb_programmer")
# block until the add_two_ints service is available
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

# Count MCB's and make sure we have the right number.
action = StringStringResponse('retry')
count_cmd = "LD_LIBRARY_PATH=" + path + " " + path + "/eccount" + " -i eth0"
expected = len(all)
try:
  while (action.str == "retry"):
    count = subprocess.call(count_cmd, shell=True)
    if count == expected:
      print "Found %s MCB's, programming" % count
      action.str = "pass"
    else:
      action = result_proxy("MCB counts don't match. Found %s, expected %s" % (count, expected))
      if action.str == "fail":
        print "Programming MCB's failed, counts don't match!"
        success = False
        sys.exit(0)
except OSError, e:
  action = result_proxy("Failed to count MCB's, cannot program.")
  success = False
  sys.exit(0)


for num in all:
  action = StringStringResponse('retry')
  print "Doing something with %s"%num
  try:
    while(action.str == "retry"):
      filename = path + "/*.bit" 

      cmd = "LD_LIBRARY_PATH=" + path + " " + path + "/fwprog" + " -i eth0 -p %s %s"%(num, filename)
      retcode = subprocess.call(cmd, shell=True)
      print "Attempted to program MCB %s" % num
      if retcode != 0:
        action = result_proxy("Programming MCB firmware failed for MCB #%s with error %d! Would you like to retry?"%(num, retcode))
        if action.str == "fail":
          print "Programming MCB firmware failed for %s!"%num
          success = False
          break
        
      else: # retcode = 0 -> success
        print "Programmed MCB %s" % num
        action.str = "pass"
  except OSError, e:
    action = result_proxy("The MCB firmware programing failed to execute.")
    success = False

if success:
  print "Programming MCB firmware finished"
  action = result_proxy("done")
 
sys.exit(0)
