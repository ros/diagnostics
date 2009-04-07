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

# Call script from here, returns 0 if correct count
count_cmd = roslib.packages.get_pkg_dir("qualification", True) + "/scripts/count_mcbs.py %s" % len(mcbs)
retcode = subprocess.call(count_cmd, shell=True)
if retcode != 0:
  print 'Unable to verify MCB count'
  success = False
  sys.exit(255)

# Configure MCB's
path = roslib.packages.get_pkg_dir("ethercat_hardware", True)
actuator_path = path + "/actuators.conf"

#wait for MCB's to initialize after being turned on
time.sleep(5)

details = ''
for name, num in mcbs:
  action = StringStringResponse('retry')
  
  try:
    while(action.str == "retry"):
      cmd = path + "/motorconf" + " -i eth0 -p -n %s -d %s -a %s" % (name, num, actuator_path)
      
      p = subprocess.Popen(cmd, stdout = subprocess.PIPE, stderr = subprocess.PIPE, shell = True)
      stdout, stderr = p.communicate()
      retcode = p.returncode

      details = 'Ran motorconf. Attempted to program MCB %s with actuator name %s. Return code: %s.\n\n' % (num, name, retcode)
      details += 'STDOUT:\n' + stdout
      if len(stderr) > 5:
        details += '\nSTDERR:\n' + stderr
      
      if retcode != 0:
        action = result_proxy("Programming MCB configuration failed for %s with return code %s! Retry?:::%s" % (name, retcode, details))
        if action.str == "fail":
          print "Programming MCB configuration failed for %s! Shutting down test!"%name
          success = False
          sys.exit(1)
          break
      else:
        print 'Configured MCB %s with actuator %s' % (num, name)
        action.str = "pass"
  except OSError, e:
    print e
    action = result_proxy("The MCB configuration program failed to execute. Press YES or NO to exit.:::%s" % details)
    success = False
    sys.exit(2)
      
if success:
  print "MCB configuration finished."
  action = result_proxy("done")
  sys.exit(0)
  
sys.exit(4)
