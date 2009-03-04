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
import rospy, sys, time
import subprocess
from optparse import OptionParser

from deprecated_srvs.srv import * 

rospy.init_node("mcb_configurer")
rospy.wait_for_service('mcb_conf_results')

result_proxy = rospy.ServiceProxy('mcb_conf_results', StringString)
parser = OptionParser()
parser.add_option("--motor=", type="string", dest="mcbs", action="append")


options, args = parser.parse_args()

mcbs = []
for args in options.mcbs:
  mcbs.append(args.split(","))

success = True

count_path = roslib.packages.get_pkg_dir("qualification", True) + "/fwprog"

# Count MCB's and make sure we have the right number.
action = StringStringResponse('retry')
count_cmd = "LD_LIBRARY_PATH=" + count_path + " " + count_path + "/eccount" + " -i eth0"
expected = len(mcbs)
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
        sys.exit(2)
except OSError, e:
  action = result_proxy("Failed to count MCB's, cannot program.")
  success = False
  sys.exit(2)

# Configure MCB's
path = roslib.packages.get_pkg_dir("ethercat_hardware", True)
actuator_path = path + "/actuators.conf"

#wait for MCB's to initialize after being turned on
time.sleep(5)

for name, num in mcbs:
  action = StringStringResponse('retry')

  try:
      while(action.str == "retry"):
        retcode = subprocess.call(path + "/motorconf" + " -i eth0 -p -n %s -d %s -a %s"%(name, num, actuator_path), shell=True)
        if retcode != 0:
            action = result_proxy("Programming MCB configuration failed for %s with return code %s! Retry?" % (name, retcode))
            if action.str == "fail":
              print "Programming MCB configuration failed for %s! Shutting down test!"%name
              success = False
              sys.exit(1)
              break

        else:
          #print retcode
          action.str ="pass"
  except OSError, e:
      action = result_proxy("The MCB configuration program failed to execute.")
      success = False
      sys.exit(1)
      
if success:
    print "Programming MCB confiuration finished"
    action = result_proxy("done")
    sys.exit(0)

sys.exit(1)
