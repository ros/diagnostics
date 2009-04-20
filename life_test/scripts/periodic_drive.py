#! /usr/bin/env python
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

# Author: Stuart Glaser
import time
import random
import roslib
roslib.load_manifest('life_test')
import rospy
from std_msgs.msg import Float64
from robot_msgs.msg import MechanismState

STRAIGHT = 0.82
ROTATION_JOINT = 'fl_caster_rotation_joint'
SPEED = 1000.0
PERIOD = 30.0
PI = 3.14159

class LastMessage():
    def __init__(self, topic, msg_type):
        self.msg = None
        rospy.Subscriber(topic, msg_type, self.callback)

    def last(self):
        return self.msg

    def callback(self, msg):
        self.msg = msg

def main():
    angle = STRAIGHT
    speed = -SPEED
    last_time = 0
    rospy.init_node('periodic_drive', anonymous=True)
    last_state = LastMessage('/mechanism_state', MechanismState)
    pub_steer = rospy.Publisher("/caster/steer_velocity", Float64)
    pub_drive = rospy.Publisher("/caster/drive_effort", Float64)
    pub_steer.publish(Float64(0.0))
    pub_drive.publish(Float64(0.0))
    print "Waiting for a mechanism_state message..."
    while not last_state.msg: pass
    while not rospy.is_shutdown():
        time.sleep(0.01)
        mech_state = last_state.last()
        rotation_state = None
        for joint_state in mech_state.joint_states:
            if joint_state.name == ROTATION_JOINT:
                rotation_state = joint_state
                break
        if not rotation_state:
            print "The %s joint was not found in the mechanism state" % ROTATION_JOINT

        # Steers the caster to be straight
        pub_steer.publish(Float64(6.0 * (angle - rotation_state.position)))

        # Drive
        if abs(rotation_state.position - angle) < 0.05:
            pub_drive.publish(Float64(speed))
        else:
            pub_drive.publish(Float64(0.0))

        if mech_state.time - last_time > (PERIOD / 2):
            speed *= -1
            if(random.random() > 0.05): # Rotate caster
                pub_drive.publish(Float64(speed))
                time.sleep(0.75)
                speed *= -1                
                if angle > PI:
                    angle -= PI
                else:
                    angle += PI
            last_time = mech_state.time


if __name__ == '__main__':
    main()
