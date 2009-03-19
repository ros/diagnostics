#!/usr/bin/env python 
# Copyright (c) 2008, Willow Garage, Inc.  
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

# Author: Kevin Watts        

PKG = "arm_life_test"

import roslib; roslib.load_manifest(PKG)
roslib.load_manifest('pr2_mechanism_controllers')
roslib.load_manifest('mechanism_control')

import sys
import os
import rospy
import random

from time import sleep
from std_msgs.msg import *
from mechanism_control import mechanism
from robot_srvs.srv import SpawnController

def xml_for():
    return "\
<controller name=\"arm_effort_controller\" type=\"JointEffortControllerNode\">\
  <joint name=\"r_elbow_flex_joint\" />\
</controller>" 

if __name__ == "__main__":
    rospy.init_node('arm_impact_test')
    rospy.wait_for_service('spawn_controller')
    spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)

    resp = spawn_controller(xml_for())
    if len(resp.ok) < 1 or not resp.ok[0]:
        print "Failed to spawn controller"
        sys.exit(1)

    pub = rospy.Publisher("arm_effort_controller/set_command", Float64)

    effort = 15

    try:
        while not rospy.is_shutdown():
            pub.publish(Float64(effort))
            sleep(1)
            effort = effort * -1
    finally:
        for i in range(1,3):
            try:
                mechanism.kill_controller('arm_effort_controller')
            except:
                print 'Failed to kill arm life controller'
