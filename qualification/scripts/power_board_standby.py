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
import rospy, sys,time
import subprocess
from optparse import OptionParser


#rospy.init_node("power_cycler")
# block until the service is available
#rospy.wait_for_service('power_board_control')

parser = OptionParser()
parser.add_option("--time=", type="int", dest="seconds", action="store", default="10")


options, args = parser.parse_args()


path = rostools.packspec.get_pkg_dir("pr2_power_board", True)
try:
  retcode = subprocess.call(path + "/scripts/send_command 0 reset", shell=True)
  if retcode != 0:
    print "Power Cycle command failed"

  retcode = subprocess.call(path + "/scripts/send_command 1 reset", shell=True)
  if retcode != 0:
    print "Power Cycle command failed"

  retcode = subprocess.call(path + "/scripts/send_command 2 reset", shell=True)
  if retcode != 0:
    print "Power Cycle command failed"


  retcode = subprocess.call(path + "/scripts/send_command 0 stop", shell=True)
  if retcode != 0:
    print "Power Cycle command failed"

  retcode = subprocess.call(path + "/scripts/send_command 1 stop", shell=True)
  if retcode != 0:
    print "Power Cycle command failed"

  retcode = subprocess.call(path + "/scripts/send_command 2 stop", shell=True)
  if retcode != 0:
    print "Power Cycle command failed"

  time.sleep(1)
  retcode = subprocess.call(path + "/scripts/send_command 2 terrible_hack_shutdown", shell=True)
  if retcode != 0:
    print "Power Cycle command failed"



except OSError, e:
  print "OSError"
