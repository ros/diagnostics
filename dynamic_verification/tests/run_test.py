#!/usr/bin/python                                                               
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
#                                                                               

# Author: Kevin Watts    

PKG = 'dynamic_verification'
import roslib
roslib.load_manifest(PKG)
roslib.load_manifest('mechanism_control')
roslib.load_manifest('mechanism_bringup')

import rospy
import sys
from time import sleep
import os

from std_msgs.msg import *
from robot_msgs.msg import PoseDot

from robot_mechanism_controllers.srv import *
from robot_mechanism_controllers import controllers
from mechanism_control import mechanism

class SendMessageOnSubscribe(rospy.SubscribeListener):
    def __init__(self, msg):
        self.msg = msg
        print "Waiting for subscriber..."
        
    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        peer_publish(self.msg)
        sleep(0.1)

#spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)
#kill_controller = rospy.ServiceProxy('kill_controller', KillController)

def xml_for_hold(name, p, i, d, iClamp):
    return """                                                                 
<controller name="%s_controller" type="JointPositionControllerNode">       
<joint name="%s_joint">                                                       
<pid p="%d" i="%d" d="%d" iClamp="%d" />                                       
</controller>""" % (name, name, p, i, d, iClamp)

def set_controller(controller, command):
    pub = rospy.Publisher('/' + controller + '/set_command', Float64, SendMessageOnSubscribe(Float64(command)))
    
    
def hold_joint(name, p, i, d, iClamp, holding):
    # Try to launch 3x                                                          
    # If launched, add to list of holding controllers and return true           
    for i in range(1,4):
        try:
            mechanism.spawn_controller(xml_for_hold(name, p, i, d, iClamp))
            holding.append(name + "_controller")
            return True
        except Exception, e:
            print "Failed to spawn holding controller %s on try %d" % (name, i)
    return False

def finished(msg):
    return msg

def main():
    # Parse sys args
    hold_jnt = str(sys.argv[1])
    position = float(sys.argv[2])
    path = str(sys.argv[3])

    xml_for_dy_ver = open(path)

    # Spawn holding controller
    holding = []

    rospy.wait_for_service('spawn_controller')
    rospy.init_node('run_dynamic_test', anonymous=True)

    try:
        hold = hold_joint(hold_jnt, 20, 3, 4, 2, holding)
        if hold:
            set_controller(hold_jnt + "_controller", position)
        
        sleep(2)
        # Spawn dynamic controller
        mechanism.spawn_controller(xml_for_dy_ver.read())
        # Find creative way to kill controller
        holding.append("r_wrist_flex_torque_controller")

        rospy.wait_for_service("/dynamic_verification_done")
        rospy.Service("/dynamic_verification_done",Empty,finished)

    finally:
        # Kill all holding controllers
        print "Releasing controllers"
        for name in holding:
            print "Releasing %s" % name
            for i in range(1,6):
                try:
                    mechanism.kill_controller(name)
                    break # Go to next controller if no exception               
                except:
                    print "Failed to kill controller %s on try %d" % (name, i)
        sys.exit(0)

        
if __name__ == '__main__':
    main()
