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

rospy.init_node("mcb_programmer")
# block until the add_two_ints service is available
rospy.wait_for_service('mcb_conf_results')

result_proxy = rospy.ServiceProxy('mcb_conf_results', StringString)
parser = OptionParser()
parser.add_option("--motor=", type="string", dest="mcbs", action="append")


options, args = parser.parse_args()

mcbs = []
for args in options.mcbs:
  mcbs.append(args.split(","))


path = rostools.packspec.get_pkg_dir("ethercat_hardware", True)
actuator_path = path + "/actuators.conf"
success = True

for name, num in mcbs:
  action = StringStringResponse('retry')

  try:
      while(action == "retry"):
        retcode = subprocess.call(path + "/motorconf" + " -i rteth0 -p -n %s -d %s -a %s"%(name, num, actuator_path), shell=True)
        if retcode != 0:
            action = result_proxy("Programming MCB confiuration failed for %s!"%name)
            if action == "fail":
              print "Programming MCB confiuration failed for %s!"%name
              #wx.MessageBox("Programming MCB confiuration failed for %s!"%name)
            elif action =="fail":
              success = False
              break

        else:
          action = "pass"
  except OSError, e:
      action = result_proxy("The MCB configuration program failed to execute.")
      success = False

if success:
    print "Programming MCB confiuration finished"
    action = result_proxy("done")
 
#return success
