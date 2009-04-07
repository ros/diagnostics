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


def main():
    expected = sys.argv[1] # Number of MCB's expected
    
    # Need to init node?    
    rospy.init_node("mcb_counter")

    result_proxy = rospy.ServiceProxy('mcb_conf_results', StringString)
    
    success = True
    
    count_path = roslib.packages.get_pkg_dir("qualification", True) + "/fwprog"
    
    # Count MCB's and make sure we have the right number.
    action = StringStringResponse('retry')
    count_cmd = "LD_LIBRARY_PATH=" + count_path + " " + count_path + "/eccount" + " -i eth0"
    details = ''
    try:
        while (action.str == "retry"):
            p = subprocess.Popen(count_cmd, stdout =subprocess.PIPE,
                                 stderr = subprocess.PIPE, shell=True)
            stdout, stderr = p.communicate()
            count = p.returncode
            details = 'Ran eccount. Expected %s MCB\'s, found %s.\n\n' % (expected, count)
            details += 'STDOUT:\n' + stdout
            if len(stderr) > 5:
                details += '\nSTDERR:\n' + stderr
                      
            if str(count) == str(expected):
                print "Found %s MCB's, programming" % count
                action.str = "pass"
                return 0
            elif count == 0:
                action = result_proxy("Found no MCB's! Check cables and power. Retry?:::%s" % details)
            elif count > 199:
                action = result_proxy("Error counting MCB's. eccount gave error code: %s. Retry?:::%s" % (count, details))
            else:
                action = result_proxy("MCB counts don't match. Found %s, expected %s. Retry?:::%s" % (count, expected, details))
                
            if action.str == "fail":
                print "Programming MCB's failed, counts don't match!"
                return 1
    
    except OSError, e:
        action = result_proxy("Failed to count MCB's, cannot program. Press OK to cancel.:::%s", details)
        return 2

if __name__ == '__main__':
    main()
