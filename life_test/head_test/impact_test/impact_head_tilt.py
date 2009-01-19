#! /usr/bin/python
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# This script brings up an effort controller runs a 500x life test on the head tilt 
# joint. Safety effort limits need to be removed. 
#
# Author: Kevin Watts

import random
CONTROLLER_NAME = "impact_head_tilt_controller_%08d" % random.randint(0,10**8-1)

import sys

import rostools
rostools.update_path('head_tilt_life_test')
import rospy
from std_msgs.msg import *
from mechanism_control import mechanism
from robot_srvs.srv import SpawnController, KillController
from time import sleep

# Use effort controller for back & forth motion
def xml_for(joint):
    return "\
<controller name=\"%s\" type=\"JointEffortControllerNode\">\
<joint name=\"%s\" />\
</controller>" % (CONTROLLER_NAME, joint)

def main():
    joint = "head_tilt_joint" # Should it say pr2_... ?

    rospy.init_node('impact_head', anonymous=True)
    rospy.wait_for_service('spawn_controller')
    spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)
    kill_controller = rospy.ServiceProxy('kill_controller', KillController)
    
    resp = spawn_controller(xml_for(joint))
    if len(resp.ok) < 1 or not ord(resp.ok[0]):
        print "Failed to spawn effort controller"
        sys.exit(1)

    pub = rospy.Publisher("/%s/set_command" % CONTROLLER_NAME, Float64)

    try:
        for i in range(1,500):
            if rospy.is_shutdown():
                break
            
            # Need to disable safety limits on effort!
            sleep(0.5)
            effort = -100; # Min effort
            pub.publish(Float64(effort))
            sleep(0.5)
            effort = 100; # Max effort
            pub.publish(Float64(effort))
    finally:
        kill_controller(CONTROLLER_NAME)
    
if __name__ == '__main__':
    main()



