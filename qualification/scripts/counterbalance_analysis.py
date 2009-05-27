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

import rospy
import sys, os, string
from time import sleep

import traceback

import matplotlib
import matplotlib.pyplot as plot
from StringIO import StringIO

import roslaunch

from qualification.msg import Plot
from qualification.srv import *

from joint_qualification_controllers.srv import * 

class CounterBalanceAnalysis:
    def __init__(self):
        rospy.init_node('cb_analysis')
        self.data_topic = rospy.Service('hold_set_data', HoldSetData, self.hold_callback)
        self.result_service = rospy.ServiceProxy('test_result', TestResult)
        
        self._joints = []
        self._joints_to_path = {}

        for arg in sys.argv[1:]:
            joint, sep, path = arg.split(',')
            self._joints_to_path[joint] = path
            
        self._joint_index = 0
        self.hold_results = {}

        self._timeout = 120

        self._test_launcher = None
        self._sent_results = False

        self._hold_data_srvs = []

        r = TestResultRequest()
        r.html_result = '<p>Test Failed.</p>'
        r.text_summary = 'Failure.'
        r.plots = []
        r.result = TestResultRequest.RESULT_FAIL

        # Roslaunch spawner for lift, after callback launch for flex
        print 'Num joints', len(self._joints)
        print 'Joints: %s' % string.join(self._joints, ', ')

        sleep(1.0)
        self.wait_for_analysis()

    def wait_for_analysis(self):
        self._waiting = False
        self._wait_time = rospy.get_time()

        try:
            while not rospy.is_shutdown():
                if self._waiting and rospy.get_time() - self._wait_time < self._timeout:
                    sleep(1.0)
                    continue

                elif self._waiting:
                    rospy.logerr('Reached timeout, killing launch')
                    # Stop launch
                    self._test_launcher.stop()
                    self._waiting = False

                if not self._waiting:
                    if self._joint_index < len(self._joints):
                        self.launch_joint()
                    elif not self._sent_results:
                        self.send_results()
                    else:
                        sleep(1.0)
        except Exception, e:
            self.test_failed_service_call(traceback.format_exc())
                    
    def send_results(self):
        html = '<p>Counterbalance Analysis</p>\n'

        for srv in self._hold_data_srvs:
            html += '<hr size="2">\n'
            html += self.hold_analysis(srv)

        r.html_result = html
        r.text_summary = 'Analysis performed. Joints: %s' % string.join(self._joints, ', ')
        r.result = TestResultRequest.RESULT_HUMAN_REQUIRED

        if not self._sent_results:
            self.result_service.call(r)
        self._sent_results = True
            
      
    def test_failed_service_call(self, except_str = ''):
        rospy.logerr(except_str)
        r = TestResultRequest()
        r.html_result = except_str
        r.text_summary = 'Caught exception, automated test failure.'
        r.plots = []
        r.result = TestResultRequest.RESULT_FAIL
        if not self._sent_results:
            self.result_service.call(r)
        self._sent_results = True


    def launch_joint(self):
        # Stop running in we're running
        if self._test_launcher:
            self._test_launcher.stop()

        joint = self._joints[self._joint_index]
        path = self._joints_to_path[joint]

        launch_text = '<launch>\n<node pkg="qualification" type="full_arm_test_spawner.py" args="%s %s" />\n</launch>\n' % (joint, path)


        self._joint_index += 1

        rospy.logerr(launch_text)

        config = roslaunch.ROSLaunchConfig()
        try:
            loader = roslaunch.XmlLoader()
            loader.load_string(launch_text, config)
            self._test_launcher = roslaunch.ROSLaunchRunner(config)
            self._test_launcher.launch()
            
        except:
            traceback.print_exc()
            self.test_failed_service_call(traceback.format_exc())
   

    def hold_callback(self, srv):
        self._hold_data_srvs.append(srv)
        self._waiting = False
        return HoldSetDataResponse()

    def hold_analysis(self, srv):
        fig = plot.figure(1)
        axes1 = fig.add_subplot(211)
        axes2 = fig.add_subplot(212)
        axes2.set_xlabel('Position')
        axes1.set_ylabel('Effort')
        axes2.set_ylabel('Effort SD')
        fig.text(.35, 0.95, srv.joint_name)

        pos_avg = []
        vel_sd = []
        effort_avg = []
        effort_sd = []

        for hold in srv.hold_data:
            pos_avg.append(numpy.average(hold.position))
            vel_sd.append(numpy.std(hold.velocity))
            effort_avg.append(numpy.average(hold.effort))
            effort_sd.append(numpy.std(hold.effort))
            
        
        axes1.plot(numpy.array(pos_avg), numpy.array(effort_avg))
        axes2.plot(numpy.array(pos_avg), numpy.array(effort_sd))

        stream = StringIO()
        plot.savefig(stream, format = 'png')
        image = stream.getvalue()
        p = qualification.msg.Plot()
        r.plots.append(p)
        p.title = srv.joint_name + '_hold_data'
        p.image = image
        p.image_format = 'png'

        # Make HTML table for each joint with all data
        html = '<H4>%s</H4>\n' % srv.joint_name
        html += '<img src="IMG_PATH/%s.png" width="640" height="480" /><br><br>\n' % p.title
        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Goal</b></td><td><b>Position</b></td><td><b>Effort</b></td><td><b>Effort SD</b></td><td><b>Velocity SD</b></td></tr>\n'
        for i in range(0, len(pos_avg)):
            goal = srv.hold_data[i].desired
            pos = numpy.average(srv.hold_data[i].position)
            eff = numpy.average(srv.hold_data[i].effort) 
            eff_sd = numpy.std(srv.hold_data[i].effort)
            vel = numpy.std(srv.hold_data[i].velocity) 

            html += '<tr><td>%s</td><td>%.2f</td><td>%.3f</td><td>%.3f</td><td>%.3f</td></tr>\n' % (goal, pos, eff, eff_sd, vel)

        html += '</table>'

        return html

            
        


if __name__ == '__main__':
    try:
        app = CounterBalanceAnalysis()
        rospy.spin()
    except Exception, e:
        traceback.format_exc()
