#! /usr/bin/python
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

# This script brings up an effort controller on your joint of choice
# and allows you to type in the desired efforts.
#
# Author: Stuart Glaser

import random

import sys
import math
import roslib
roslib.load_manifest('robot_mechanism_controllers')
roslib.load_manifest('mechanism_control')
import rospy
from std_msgs.msg import *
from mechanism_control import mechanism
from robot_srvs.srv import SpawnController, KillController


def controller_name(joint):
    return "sinesweep_controller_%s"%joint

def xml_for(joint):
    return "\
<controller name=\"%s\" type=\"JointEffortControllerNode\">\
<joint name=\"%s\" />\
</controller>" % (controller_name(joint), joint)

def main():
    if len(sys.argv) < 6:
        print "Usage:  signsweep.py Amplitude Freq_start Freq_stop Duration joint1 joint2 joint3 ... jointn"
        sys.exit(1)
    A = float(sys.argv[1])
    f1 = float(sys.argv[2])
    f2 = float(sys.argv[3])
    duration = float(sys.argv[4])
    num_joints = len(sys.argv)-5
    joints = []
    for i in range(0,num_joints):
        joints.append(sys.argv[i+5])
        print "Using joint",joints[i]

    rospy.init_node('effect', anonymous=True)
    rospy.wait_for_service('spawn_controller')
    spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)
    kill_controller = rospy.ServiceProxy('kill_controller', KillController)

    pub = []
    for j in joints:
        resp = spawn_controller(xml_for(j))
        if len(resp.ok) < 1 or not resp.ok[0]:
            print "Failed to spawn sinesweep controller for joint ",j
            sys.exit(1)
        pub.append(rospy.Publisher("%s/set_command" % controller_name(j), Float64))

    time_start = rospy.get_rostime()
    time = 0.0


    while (time < duration and not rospy.is_shutdown()):
        time = (rospy.get_rostime() - time_start).to_seconds()
        f = f1 + (f2-f1)*time/duration
        effort = A*math.sin(f*2*math.pi*time)
        for p in pub:        
            p.publish(Float64(effort))
    
    for j in joints:
        kill_controller(controller_name(j))


if __name__ == '__main__':
    main()
