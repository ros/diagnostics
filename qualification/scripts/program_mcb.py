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
rostools.update_path('qualification')
import rospy, sys
import subprocess
from optparse import OptionParser

from std_srvs.srv import * 

print "Starting"

rospy.init_node("mcb_programmer")
# block until the add_two_ints service is available
print "Waiting"
rospy.wait_for_service('mcb_conf_results')

result_proxy = rospy.ServiceProxy('mcb_conf_results', StringString)
parser = OptionParser()
parser.add_option("--wg005=", type="string", dest="wg005", action="append")
parser.add_option("--wg006=", type="string", dest="wg006", action="append")


options, args = parser.parse_args()

path = rostools.packspec.get_pkg_dir("qualification", True) + "/fwprog"

success = True

all = []
if options.wg005:
  if options.wg006:
    all = options.wg005 + options.wg006
  else:
    all = options.wg005
else:
  all = options.wg006

for num in all:
  action = StringStringResponse('retry')
  print "Doing something with %s"%num
  try:
      while(action.str == "retry"):
        filename = path + "/hecat-408.bit"
        if options.wg006:
          if num in options.wg006:
            filename = path + "/gripper-303.bit"


        cmd = "LD_LIBRARY_PATH=" + path + " " + path + "/fwprog" + " -i rteth0 -p %s %s"%(num, filename)
        action = result_proxy("Confirm Programming %s: \n%s"%(num,cmd))
        if action.str == "fail":
          break
        retcode = subprocess.call(cmd, shell=True)
        if retcode != 0:
            action = result_proxy("Programming MCB firmware failed for %s with error %d!"%(num, retcode))
            if action.str == "fail":
              print "Programming MCB firmware failed for %s!"%num
              success = False
              break

        else:
          print retcode
          action.str ="pass"
  except OSError, e:
      action = result_proxy("The MCB firmware programing failed to execute.")
      success = False

if success:
    print "Programming MCB firmware finished"
    action = result_proxy("done")
 
sys.exit(0)
