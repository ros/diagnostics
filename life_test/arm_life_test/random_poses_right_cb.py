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


import roslib; roslib.load_manifest('life_test')

import random, time
import rospy
from std_msgs.msg import *
from robot_msgs.msg import *

pub = rospy.Publisher('/r_arm_cartesian_pose_controller/command', PoseStamped)
#pub = rospy.Publisher('/cartesian_trajectory_right/command', PoseStamped)


def p(x, y, z, rx, ry, rz, w):
  m = PoseStamped()
  m.header.frame_id = 'torso_lift_link'
  m.header.stamp = rospy.get_rostime()
  m.pose.position.x = x
  m.pose.position.y = y
  m.pose.position.z = z
  m.pose.orientation.x = rx
  m.pose.orientation.y = ry
  m.pose.orientation.z = rz
  m.pose.orientation.w = w
  pub.publish(m)
rospy.init_node('pub', anonymous=True)

POINTS_UP = [
  (0.5, -0.5, 2.6, 0.5, -0.7, 0.0, 0.7),
  (0.6, -0.2, 2.6, 0.0, -0.7, 0.0, 0.7),
  (0.2, -0.8, 2.6, 0.0, -0.7, 0.0, 0.7),
  (0.4, -0.4, 2.6, 0.5, 0.25, 0.0, 0.25),
]

POINTS_DN = [
  (0.6, -0.5, -0.4, 0.0, -0.7, 0.0, 0.7),
  (0.8, -0.2, -0.4, 0.0, -0.7, 0.0, 0.7),
  (0.5, -0.8, -0.4, 0.0, -0.7, 0.0, 0.7),
  (0.5, -0.4, -0.4, 0.5, 0.25, 0.0, 0.25),
 ]

# Alternates between up and down to work CB
while not rospy.is_shutdown():
  for i in range(15):
    time.sleep(random.uniform(0.1, 0.1))
    p(*POINTS_UP[random.randint(0, len(POINTS_UP)-1)])

  for i in range(10):
    time.sleep(random.uniform(0.1, 0.1))
    p(*POINTS_DN[random.randint(0, len(POINTS_DN)-1)])
