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


import rostools; rostools.update_path('pr2_mechanism_controllers')
rostools.update_path('std_msgs')
rostools.update_path('rospy')

import random, time
import rospy
from std_msgs.msg import *
from robot_msgs.msg import *

pub = rospy.Publisher('/arm_position/set_command', PointStamped)

def p(x, y, z):
  m = PointStamped()
  m.header.frame_id = 'torso_link'
  m.point.x = x
  m.point.y = y
  m.point.z = z
  pub.publish(m)

rospy.init_node('pub', anonymous=True)

POINTS = [
  (0.9, -0.7, 1.5),
  (0.9, -0.7, 2),
  (1.3,  0.7, 1.3),
  (0.8,  0.7, 1.3)
]

while not rospy.is_shutdown():
  time.sleep(random.uniform(0.1, 0.5))
  p(*POINTS[random.randint(0, len(POINTS)-1)])
