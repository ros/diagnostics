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

# Author: Melonee Wise

PKG = "life_test"

import roslib; roslib.load_manifest(PKG)

import sys
import os
import string
from time import sleep
import random
import rospy
from robot_msgs.msg import JointCmd

def point_head_client(pan, tilt):

    head_angles.publish(JointCmd(['head_pan_joint', 'head_tilt_joint'],[0.0,0.0],[pan, tilt],[0.0, 0.0],[0.0, 0.0]))
    sleep(0.5)

if __name__ == "__main__":

   head_angles = rospy.Publisher('head_controller/set_command_array', JointCmd)
   rospy.init_node('head_commander', anonymous=True)
   sleep(1)

   while 1:
       #point_head_client(0.0, -0.4)
       pan = random.uniform(-2.7, 2.7)
       print pan
       tilt = random.uniform(-0.5, 1.35)
       print tilt
       #point_head_client(0.0, 1.2)
       point_head_client(pan, tilt)

