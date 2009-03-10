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

# Author: Kevin Watts

PKG = "head_life_test"

import roslib; roslib.load_manifest(PKG)
roslib.load_manifest('pr2_mechanism_controllers')
# Loads interface with the robot.        
roslib.load_manifest('mechanism_control')

import sys
import os
import string
from time import sleep
import random
import rospy
from std_msgs import *
from robot_msgs.msg import JointCmd
from pr2_mechanism_controllers.srv import *

from mechanism_control import mechanism


def point_head_client(pan, tilt):
    head_angles.publish(JointCmd(['head_pan_joint', 'head_tilt_joint'],[0.0,0.0],[pan, tilt],[0.0, 0.0],[0.0, 0.0]))
    sleep(0.5)

if __name__ == "__main__":
   # spawn head test controller from xml file in pkg
   head_controller_path = roslib.packages.get_pkg_dir('head_life_test') + '/head_controller.xml'
   xml_for_head = open(head_controller_path)
   mechanism.spawn_controller(xml_for_head.read())

   # run laser controller
   # based off pr2_mechanism_controllers/control_laser.py
   
   rospy.wait_for_service('laser_controller/set_profile')
   s = rospy.ServiceProxy('laser_controller/set_profile', SetProfile)
   resp = s.call(SetProfileRequest(0.0, 0.0, 0.0, 0.0, 4, 1.25, 0.95, 0.25))

   head_angles = rospy.Publisher('head_controller/set_command_array', JointCmd)
   rospy.init_node('head_commander', anonymous=True)
   sleep(1)

   try:
       while not rospy.is_shutdown():
           pan = random.uniform(-2.7, 2.7)
           tilt = random.uniform(-0.3, 0.8)
           point_head_client(pan, tilt)
   except Exception, e:
       print 'Caught exception running controllers'
       print e
   finally:
       print 'Killing controllers'
       for i in range(1,3):
           try:
               mechanism.kill_controller('head_controller')
           except:
               print 'Failed to kill head controller'

       for i in range (1,3):
           try:
               mechanism.kill_controller('laser_controller')
           except:
               print 'Failed to kill laser controller'


    

