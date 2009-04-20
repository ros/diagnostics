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

PKG = "life_test"

import roslib; roslib.load_manifest(PKG)

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

if __name__ == "__main__":
   # spawn head test controller from xml file in pkg
   controller_path = roslib.packages.get_pkg_dir('life_test') + '/laser_tilt_test/life_test/controllers.xml'
   xml_for = open(controller_path)
   mechanism.spawn_controller(xml_for.read())

   # run laser controller with sine profile
   # based off pr2_mechanism_controllers/control_laser.py
   
   rospy.wait_for_service('laser_controller/set_profile')
   s = rospy.ServiceProxy('laser_controller/set_profile', SetProfile)
   resp = s.call(SetProfileRequest(0.0, 0.0, 0.0, 0.0, 4, 0.25, 1.25, 0.25))

   rospy.init_node('hokuyo_commander', anonymous=True)
   sleep(1)

   try:
      rospy.spin()
   except Exception, e:
      print 'Caught exception running controllers'
      print e
   finally:
      print 'Killing controllers'
      for i in range (1,3):
         try:
            mechanism.kill_controller('laser_controller')
         except:
            print 'Failed to kill laser controller'


    

