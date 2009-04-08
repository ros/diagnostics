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
import sys
from std_msgs.msg import *
from robot_msgs.msg import *

pub = rospy.Publisher('/r_arm_cartesian_wrench_controller/command', Wrench)

def p(fx, fy, fz, tx, ty, tz):
  m = Wrench()
  m.force.x = fx
  m.force.y = fy
  m.force.z = fz
  m.torque.x = tx
  m.torque.y = ty
  m.torque.z = tz
  pub.publish(m)
rospy.init_node('pub', anonymous=True)


def main():
    if len(sys.argv) < 3:
        print "Usage:  random_wrenches.py <magnitude [N]> <magnitude [Nm]>"
        sys.exit(1)
    if not sys.argv[1].isdigit():
        print "give integer as paramter"
        sys.exit(1)
    if not sys.argv[2].isdigit():
        print "give integer as paramter"
        sys.exit(1)
    force  = int(sys.argv[1])
    torque = int(sys.argv[2])
    print "magnitude force %f"%force
    print "magnitude torque %f"%torque
    POINTS = [
      ( (force),  (force),  (force),  -(torque),  -(torque),  -(torque)),
      (-(force),  (force),  (force),   (torque),  -(torque),  -(torque)),
      ( (force),  (force),  (force),  -(torque),   (torque),  -(torque)),
      (-(force),  (force),  (force),   (torque),   (torque),  -(torque)),
      ( (force), -(force), -(force),  -(torque),  -(torque),   (torque)),
      (-(force), -(force), -(force),   (torque),  -(torque),   (torque)),
      ( (force), -(force), -(force),  -(torque),   (torque),   (torque)),
      (-(force), -(force), -(force),   (torque),   (torque),   (torque))
    ]

    while not rospy.is_shutdown():
      time.sleep(random.uniform(0.1, 0.4))
      p(*POINTS[random.randint(0, len(POINTS)-1)])

if __name__ == '__main__':
    main()
