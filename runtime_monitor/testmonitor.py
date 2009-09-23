#!/usr/bin/env python
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

## A basic node to listen to and display incoming diagnostic messages

import roslib
roslib.load_manifest('runtime_monitor')

import sys, time 
import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue, DiagnosticString


NAME = 'test_runtime_broadcaster'



def loop(pub):
        stat = []
        for c in range(0,4):
            floatval = []
            strval = []
            floatval.append(KeyValue(c, "Time Remaining Controller %d"%c))
            floatval.append(KeyValue(c+.1, "Average charge percent Controller %d"%c))
            floatval.append(KeyValue(c+.2, "Current Controller %d"%c))
            floatval.append(KeyValue(c+.2, "Voltage Controller %d"%c))
            strval.append(DiagnosticString("String Value","Controller String Label"))
            stat.append(DiagnosticStatus(c, "IBPS %d"%c, "All good", floatval,strval))
            ## @todo make the status string represent errors etc

            for b in range(0,4):
                bval = []
                bstrval = []
                bval.append(KeyValue(c+b*.01+.1, "present (0,1)"))
                bval.append(KeyValue(c+b*.01+.2, "charging (0,1)"))
                bval.append(KeyValue(c+b*.01+.3, "supplying power (0,1)"))
                bstrval.append(DiagnosticString("String Value","Controller, String Label"))
		
		for a in range(0,100):
			bstrval.append(DiagnosticString("Lots of reeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaallllllllllllllllllllllllllllllllllllllyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy long string values!\nand\nnewlines", "Test"))
                stat.append(DiagnosticStatus(b, "Smart Battery %d.%d"%(c,b), "All good", bval, bstrval))
                ## @todo make the status string represent errors etc
                            

            
        out = DiagnosticArray(None, stat)
        pub.publish(out)
        print "Published"



    
def listener():
    pub = rospy.Publisher("/diagnostics", DiagnosticArray)
    rospy.init_node(NAME, anonymous=True)
    while not rospy.is_shutdown():
        loop(pub)
        time.sleep(1)
        
if __name__ == '__main__':
    try:
        listener()
    except KeyboardInterrupt, e:
        pass
    print "exiting"
