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
import subprocess
from optparse import OptionParser

from std_msgs.msg import *
from robot_srvs.srv import *
from std_srvs.srv import *

from robot_mechanism_controllers.srv import *
from robot_mechanism_controllers import controllers
from mechanism_control import mechanism

spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)
kill_controller = rospy.ServiceProxy('kill_controller', KillController) 

class SendMessageOnSubscribe(rospy.SubscribeListener):
    def __init__(self, msg):
        self.msg = msg
        print "Waiting for subscriber..."

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        peer_publish(self.msg)
        time.sleep(0.1)


def xml_for_hold(name, p, i, d, iClamp):
    return """
<controller name="%s_hold" type="JointPositionControllerNode">
  <joint name="%s_joint">
  <pid p="%d" i="%d" d="%d" iClamp="%d" />
</controller>""" % (name, name, p, i, d, iClamp)

def hold_joint(name, p, i, d, iClamp, holding):
    try:
        resp = spawn_controller(xml_for_hold(name, p, i, d, iClamp))
        if ord(resp.ok[0]) != 0:
            holding.append(resp.name[0])
            return True
        else:
            rospy.logerr('Failed to spawn controller %s' % resp.name)
            rospy.logerr('Spawner error: %s' % resp.error)
    except Exception, e:
        print "Failed to spawn holding controller %s" % name
        print xml_for_hold(name, p, i, d, iClamp)
        
    return False

def set_controller(controller, command):
    pub = rospy.Publisher(controller + '/set_command', Float64,
                              SendMessageOnSubscribe(Float64(command)))

def hold_arm(side, pan_angle, holding):
    if hold_joint("%s_gripper" % side, 15, 0, 1, 1, holding):
        set_controller("%s_gripper_hold" % side, float(0.0))
    
    if hold_joint("%s_wrist_roll" % side, 12, 3, 1, 1, holding):
        set_controller("%s_wrist_roll_hold" % side, float(0.0))
        
    if hold_joint("%s_wrist_flex" % side, 12, 3, 1, 1, holding):
        set_controller("%s_wrist_flex_hold" % side, float(1.0))

    if hold_joint("%s_forearm_roll" % side, 20, 5, 2, 2, holding):
        set_controller("%s_forearm_roll_hold" % side, float(0.0))

    if hold_joint("%s_elbow_flex" % side, 35, 10, 2, 2, holding):
        set_controller("%s_elbow_flex_hold" % side, float(-0.5))

    if hold_joint("%s_upper_arm_roll" % side, 20, 2, 1.0, 1.0, holding):
        set_controller("%s_upper_arm_roll_hold" % side, float(0.0))

    if hold_joint("%s_shoulder_lift" % side, 40, 7, 2, 4, holding):
        set_controller("%s_shoulder_lift_hold" % side, float(0.5))

    if hold_joint("%s_shoulder_pan" % side, 50, 6, 8, 4, holding):
        set_controller("%s_shoulder_pan_hold" % side, float(pan_angle))

def main():
    if len(sys.argv) < 3:
        print "Can't load arm, need <joint> <controller_path>"
        sys.exit(1)

    # Pull side (l or r) from param server
    side = rospy.get_param("full_arm_test/side")

    rospy.init_node('arm_test_spawner_' + side, anonymous=True)
    
    try:
        joint = side + sys.argv[1]
        controller_file = open(sys.argv[2])
        # Put side in to controller xml string
        controller_xml = controller_file.read() % side 
        controller_file.close()
        
        holding = []
    
        rospy.wait_for_service('spawn_controller')
        
        print 'Raising torso'
        if hold_joint("torso_lift", 2000000, 0, 1000, 1200):
            set_controller("torso_lift_hold", float(0.30))

        print 'Holding arms'
        # Hold both arms in place
        hold_arm('r', -1.2, holding)
        hold_arm('l', 1.2, holding)
        
        print 'Launching joint controller %s' % joint
        # Kill controller for given joint
        kill_controller(joint + '_hold')

        if joint + '_hold' in holding:
            holding.remove(joint + '_hold')
        else:
            print 'Joint %s is not being held' % joint
        
        time.sleep(1.0)
        
        print 'Spawning test controller'
        # Spawn test controller and run test
        resp = spawn_controller(controller_xml)
        
        if len(resp.ok) != 1 or resp.ok[0] != chr(1):
            rospy.logerr('Failed to spawn test controller')
            rospy.logerr('Controller XML: %s' % controller_xml)
            sys.exit(2)

        holding.append(resp.name[0])
        
        print 'Test controller is up, running test'
        while not rospy.is_shutdown():
            time.sleep(0.5)

    # Kill everything
    finally:
        try:
            # Hack, kill test controller first to make sure next test works
            kill_controller('test_controller')
        except:
            pass

        for name in holding:
            for i in range(0,5):
                try:
                    rospy.logout("Trying to kill %s" % name)
                    resp = kill_controller(name)
                    if (ord(resp) != 0):
                        rospy.logout("Succeeded in killing %s" % name)
                        break
                except rospy.ServiceException:
                    rospy.logerr("ServiceException while killing %s" % name)

if __name__ == '__main__':
    main()
