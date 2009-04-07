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

PKG = "dynamic_verification"

import roslib; roslib.load_manifest(PKG)

import numpy
import math
from robot_srvs.srv import DynamicResponseData

import sys
import os
import datetime
import string
from time import sleep
from std_msgs import *
from std_srvs.srv import *

import csv 

import rospy

class App:
    def __init__(self):
        rospy.init_node("DynamicVerifier", anonymous=True)
        self.data_topic = rospy.Service("/dynamic_response_data", DynamicResponseData, self.OnData)
        rospy.spin()

    def OnData(self, req):
        # Store data 
        self.data = req
        print "Received data %s" % self.data.test_name
        if self.data.test_name == "constant_torque":
            self.write_const_torque()
        elif self.data.test_name == "frequency_response":
            self.wrist_freq_resp()
        else:
            print 'Can\'t write data, unknown type: %s' % self.data.test_name

    def dummy(self,req):
        print "do nothing"

    def write_const_torque(self):
        # write to /results/joint_name/date_time file
        time_str = datetime.datetime.now().strftime("%Y%m%d_%I%M")
        
        joint_name = self.data.joint_name

        path = roslib.packages.get_pkg_dir("dynamic_verification", True) + "/results/const_torque/" + joint_name

        f = open(path + '/' + time_str + '.csv', 'w')

        data_file = csv.writer(f)

        # Store data as a csv
        time     = numpy.array(self.data.time)
        cmd      = numpy.array(self.data.cmd)
        effort   = numpy.array(self.data.effort)
        position = numpy.array(self.data.position)
        velocity = numpy.array(self.data.velocity)

        # See if there's an easy way to convert to .csv file
        datafile = ''
        for i in range(0, time.size):
            tm = str(time[i])
            cm = str(cmd[i])
            ef = str(effort[i])
            po = str(position[i])
            ve = str(velocity[i])
            #data_file.write_row(tm, cm, ef, po, ve)

            state_string = `time[i]` + ',' + `cmd[i]` + ',' + `effort[i]` + ',' + `position[i]` + ',' + `velocity[i]` + '\n'
            datafile += state_string

        f.write(datafile)

        print 'Wrote file as results/const_torque/%s/%s.csv' % (joint_name, time_str)

        done_service = rospy.ServiceProxy("/dynamic_verification_done",Empty)
        done_service.call(Empty())
    
    def write_freq_resp(self):
        # write to /results/joint_name/date_time file
        time_str = datetime.datetime.now().strftime("%Y%m%d_%I%M")
        
        joint_name = self.data.joint_name

        path = roslib.packages.get_pkg_dir("dynamic_verification", True) + "/results/freq_resp/" + joint_name

        f = open(path + '/' + time_str + '.csv', 'w')

        data_file = csv.writer(f)

        # Store data as a csv
        time     = numpy.array(self.data.time)
        cmd      = numpy.array(self.data.cmd)
        effort   = numpy.array(self.data.effort)
        position = numpy.array(self.data.position)
        velocity = numpy.array(self.data.velocity)

        # See if there's an easy way to convert to .csv file
        for i in range(0, time.size):
            data_file.write_row(`time[i]`,`cmd[i]`,`effort[i]`,`position[i]`,`velocity[i]`)

        print 'Wrote file as results/freq_resp/%s/%s.csv' % (joint_name, time_str)


if __name__ == "__main__":
    try:
        app = App()
        rospy.spin()
    except Exception, e:
        print e
    
    print 'Quitting'
