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

from robot_srvs.srv import SpawnController, KillController

from mechanism_control import mechanism

def main():
    if len(sys.argv) < 2:
        print "Can't load test XML file."
        sys.exit(1)

    rospy.init_node('caster_test_spawner_node')

    side = rospy.get_param("caster_test/side")

    try:
        rospy.wait_for_service('spawn_controller')
        spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)
        kill_controller = rospy.ServiceProxy('kill_controller', KillController) 

        controller_file = open(sys.argv[1])
        controller_xml = controller_file.read() % side

        controller_file.close()

        resp = spawn_controller(controller_xml)
        
        if len(resp.ok) != 1 or resp.ok[0] != chr(1):
            rospy.logerr('Failed to spawn test controller')
            rospy.logerr('Controller XML: %s' % controller_xml)
            sys.exit(2)

        while not rospy.is_shutdown():
            time.sleep(0.5)
    finally:
        for i in range(3):
            try:
                rospy.logout("Trying to kill test_controller")
                resp = kill_controller('test_controller')
                if resp.ok[0] == chr(1):
                    rospy.logout("Succeeded in killing test_controller")
                    break
            except rospy.ServiceException:
                rospy.logerr("ServiceException while killing test_controller")


if __name__ == '__main__':
    main()
