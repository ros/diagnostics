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

# Puts random efforts on the wrist flex and roll at 2 Hz
#
# Author: Kevin Watts

##@package life_test
#
# @mainpage
# @htmlinclude manifest.xml
#
# @section usage Usage
# @verbatim $ run_test.launch @endverbatim
#
# @par Description
# @verbatim
# This program runs a impact life test on the head tilt, pan and laser tilt. 500 cycles, with effort safety removed.
# @endverbatim
 
import sys

import random

import roslib
roslib.load_manifest('life_test') 
import rospy
from std_msgs.msg import *
from mechanism_control import mechanism
from robot_srvs.srv import SpawnController, KillController
from time import sleep

spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)
kill_controller = rospy.ServiceProxy('kill_controller', KillController)

pub_flex = rospy.Publisher("wrist_flex_effort/command", Float64)
pub_grip = rospy.Publisher("grip_effort/command", Float64)
pub_roll = rospy.Publisher("wrist_roll_effort/command", Float64)

## Create XML code for controller on the fly
def xml_for_flex():
    return  "\
<controller name=\"wrist_flex_effort\" type=\"JointEffortControllerNode\">\
  <joint name=\"r_wrist_flex_joint\" />\
</controller>"

def xml_for_roll():
    return  "\
<controller name=\"wrist_roll_effort\" type=\"JointEffortControllerNode\">\
  <joint name=\"r_wrist_roll_joint\" />\
</controller>" 

def xml_for_grip():
    return  "\
<controller name=\"grip_effort\" type=\"JointEffortControllerNode\">\
  <joint name=\"r_gripper_joint\" />\
</controller>" 




def main():
    rospy.init_node('wrist_test', anonymous=True)
    rospy.wait_for_service('spawn_controller')
             
    resp = spawn_controller(xml_for_flex())
    if len(resp.ok) < 1 or not ord(resp.ok[0]):
        rospy.logerr("Failed to spawn effort controller")
        print xml_for_flex()
        sys.exit(100)
    else:
        print "Spawned flex controller successfully"


    resp = spawn_controller(xml_for_roll())
    if len(resp.ok) < 1 or not ord(resp.ok[0]):
        rospy.logerr("Failed to spawn effort controller roll")
        print xml_for_roll()
        sys.exit(101)
    else:
        print "Spawned flex controller successfully"


    resp = spawn_controller(xml_for_grip())
    if len(resp.ok) < 1 or not ord(resp.ok[0]):
        rospy.logerr("Failed to spawn effort controller roll")
        print xml_for_grip()
        sys.exit(102)
    else:
        print "Spawned grip controller successfully"

    effort_grip = -100 
    effort_flex = 4
    effort_roll = 4
        
    try:
        while not rospy.is_shutdown():
            if random.randint(0, 1) == 1:
                effort_flex = effort_flex * -1

            if random.randint(0, 1) == 1:
                effort_roll = effort_roll * -1
                
            pub_grip.publish(Float64(effort_grip))
            pub_flex.publish(Float64(effort_flex))
            pub_roll.publish(Float64(effort_roll))

            sleep(0.3)

    finally:
        kill_controller('grip_effort')
        kill_controller('wrist_flex_effort')
        kill_controller('wrist_roll_effort')
        sleep(1)
        sys.exit(0)
    
if __name__ == '__main__':
    main()
