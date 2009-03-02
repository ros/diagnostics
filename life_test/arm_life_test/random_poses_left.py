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


import roslib; roslib.load_manifest('arm_life_test')
roslib.load_manifest('rospy')

import random, time
import rospy
from std_msgs.msg import *
from robot_msgs.msg import *

#pub = rospy.Publisher('/cartesian_pose/command', PoseStamped)
pub = rospy.Publisher('/cartesian_trajectory_left/command', PoseStamped)


def p(x, y, z, rx, ry, rz, w):
  m = PoseStamped()
  m.header.frame_id = 'base_link'
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

POINTS = [
  (0.5, 0.5, 0.8, 0.5, 0.0, 0.0, 0.5),
  (0.6, 0.2, 0.4, 0.0, 0.0, 0.5, 0.5),
  (0.2, 0.8, 0.4, 0.0, 0.5, 0.0, 0.5),
  (0.5, 0.5, 1.2, 0.5, 0.0, 0.0, 0.5),
  (0.6, 0.2, 1.2, 0.0, 0.0, 0.5, 0.5),
  (0.2, 0.8, 1.2, 0.0, 0.5, 0.0, 0.5),
]

while not rospy.is_shutdown():
  time.sleep(random.uniform(0.1, 0.1))
  p(*POINTS[random.randint(0, len(POINTS)-1)])
