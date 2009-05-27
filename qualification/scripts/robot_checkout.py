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

# Author: Kevin Watts


# Next step: compare against known joints and actuators

import roslib
roslib.load_manifest('qualification')

import rospy
import sys, os
from time import sleep, mktime

import threading
from datetime import datetime

import wx

from robot_msgs.msg import DiagnosticMessage
from qualification.msg import Plot
from qualification.srv import *

from joint_qualification_controllers.srv import *

import traceback


def level_cmp(a, b):
    if a._level == b._level:
        return cmp(a._name, b._name)

    return cmp(b._level, a._level)

class DiagnosticItem:
    def __init__(self, name, level, message):
        self._name = name
        self._level = level
        self._message = message
        self._update_time = mktime(datetime.now().timetuple()) # Track update time

    def check_stale(self):
        if mktime(datetime.now().timetuple())- self._update_time > 3.0:
            self._level = 3

    def update_item(self, level, message):
        self._update_time = mktime(datetime.now().timetuple())
        self._level = level
        self._message = message

class RobotCheckout:
    def __init__(self):
        rospy.init_node('robot_checkout')
        self.robot_data = rospy.Service('robot_checkout', RobotData, self.on_robot_data)
        self.result_srv = rospy.ServiceProxy('test_result', TestResult)
        self.has_sent_result = False

        self.diagnostics = rospy.Subscriber('diagnostics', DiagnosticMessage, self.on_diagnostic_msg)

        self.visual_srv = rospy.Service('visual_check', ScriptDone, self.on_visual_check)
        

        rospy.logout('Subscribed to diagnostics, advertised robot data')
        
        self._calibrated = False
        self._joints = {}
        self._joints_ok = False
        
        self._actuators = {}
        self._has_robot_data = False
        self._has_visual_check = False

        self._mutex = threading.Lock()
        self._messages = []
        self._name_to_diagnostic = {}
        
        self._joint_sum = 'No joint data. '
        self._act_sum = 'No actuator data. '
        self._joint_html = '<p>No joint data.</p><hr size="2">\n'
        self._act_html = '<p>No actuator data.</p><hr size="2">\n'

        self._visual_sum = 'No visual check. '
        self._visual_html = '<p>No response from visual verification!</p>\n'

        self._joints_ok = False
        self._acts_ok = False
        self._is_ok = False
        self._visual_ok = False

        self._timeout = True
        self._check_time = 0
        
        sleep(1)
        self.wait_for_data()



    def send_failure_call(self, caller = 'No caller', except_str = ''):
        if self.has_sent_result:
            rospy.logerr('Wanted to send failure call after result sent')
            return

        r = TestResultRequest()
        r.html_result = '<p><b>Exception received during %s.</b></p>\n<p><b>Exception:</b> %s</p>\n' % (caller, except_str)
        r.text_summary = 'Exception during %s.' % caller
        r.plots = []
        r.result = r.RESULT_HUMAN_REQUIRED # TODO r.RESULT_FAIL
        try:
            rospy.wait_for_service('test_result', 10)
            self.result_srv.call(r)
            self.has_sent_result = True
        except Exception, e:
            rospy.logerr('Caught exception sending failure call! %s' % traceback.format_exc())
            

    def wait_for_data(self):
        try:
            rospy.logout('Waiting for diagnostics')
            # Wait at least 5 seconds for data
            for i in range(0, 10):
                if not rospy.is_shutdown():
                    sleep(0.5)
                    
            rospy.logout('Waiting for joint calibration')
            # Now start checking for robot data, done if we have it
            while True:
                if not rospy.is_shutdown():
                    if self._has_robot_data and self._has_visual_check:
                        self.checkout_robot()
                    sleep(0.5)
                        
            #self.checkout_robot()
        except Exception, e:
            self.send_failure_call('wait_for_data', traceback.format_exc())

    def on_diagnostic_msg(self, message):
        try:
            for stat in message.status:
                if stat.name not in self._name_to_diagnostic:
                    self._name_to_diagnostic[stat.name] = DiagnosticItem(stat.name, stat.level, stat.message)
                    
                else:
                    self._name_to_diagnostic[stat.name].update_item(stat.level, stat.message)
        except Exception, e:
            rospy.logerr('Caught exception processing diagnostic msg.\nEx: %s' % str(e))
            self.send_failure_call('on_diagnostic_msg', traceback.format_exc())

    def on_visual_check(self, srv):
        rospy.logerr('Got visual check')
        self._has_visual_check = True
        
        if srv.result == ScriptDoneRequest.RESULT_OK:
            self._visual_ok = True
            self._visual_sum = 'Visual: OK. '
            self._visual_html = '<p>Visual Verification Succeeded.</p>\n'
        else:
            self._visual_ok = False
            self._visual_html = '<p>Visual Verification Failed. '
            if srv.result == ScriptDoneRequest.RESULT_FAIL:
                self._visual_sum = 'Visual: FAIL. '
                self._visual_html += 'Operator recorded failure. Message: %s</p>\n' % srv.failure_msg
            else:
                self._visual_sum = 'Visual: ERROR. '
                self._visual_html += 'Visual verifier reported error!</p>\n'
            self._visual_html += '<p>Failure data:<br>%s</p>\n' % srv.failure_msg

        return ScriptDoneResponse()
    
            

    def on_robot_data(self, srv):
        rospy.logerr('Got robot data service')
        self._has_robot_data = True
        
        if srv.test_time > 0:
            self._timeout = False
     
        self._check_time = srv.test_time
        self.joint_data(srv.joint_data)
        self.act_data(srv.actuator_data)
            
        return RobotDataResponse()

    def process_diagnostics(self):
        # Sort diagnostics by level
        rospy.logout('Sorting diagnostic messages by level')
        diagnostics = dict.values(self._name_to_diagnostic)
        for diag in diagnostics:
            diag.check_stale()

        diagnostics.sort(level_cmp)

        # Counts number by status
        stat_count = { 3: 0, 0: 0, 1: 0, 2: 0}

        level_dict = { 3: 'Stale', 0: 'OK', 1: 'Warn', 2: 'Error' }

        table = '<table border=\"1\" cellpadding=\"2\" cellspacing=\"0\">\n'
        table += '<tr><td><b>Name</b></td><td><b>Level</b></td><td><b>Message</b></td></tr>\n'

        rospy.logout('Outputting diagnostics')
        for diag in diagnostics:
            level = level_dict[diag._level]
            table += '<tr><td>%s</td><td>%s</td><td>%s</td></tr>\n' % (diag._name, level, diag._message)
            stat_count[diag._level] = stat_count[diag._level] + 1
        table += '</table>\n'
            
        if stat_count[2] == 0 and stat_count[1] == 0 and stat_count[3] == 0 and len(diagnostics) > 0:
            summary = 'Diagnostics: OK. '
            self._is_ok = True
        else:
            if len(diagnostics) == 0:
                summary = 'No diagnostics received! '
                self._is_ok = False
            else:
                summary = 'Diagnostics FAIL: %s errors, %s warnings, %s stale item. ' % (stat_count[2], stat_count[1], stat_count[3])
                self._is_ok = False
        
        html = '<p><b>Diagnostic Data</b></p><p>%s</p><br>\n' % summary 
        html += table

        return summary, html

    def checkout_robot(self):
        try:
            html = '<p><b>Robot Checkout Test</b></p><br>\n'
                        
            diag_sum, diag_html = self.process_diagnostics()
            summary = ''
            
            if not self._has_robot_data:
                summary += 'No robot data received! '
                html += '<p><b>No robot data received!</b> CheckoutController might have had an error.</p>\n'
            
            if self._timeout and self._has_robot_data:
                summary += 'Test timed out. '
                html += '<p><b>Timeout in robot checkout controller! Check Time: %2f</b></p>\n' % self._check_time
            else:
                html += '<p>Time to complete check: %.3fs.</p>\n' % self._check_time
            summary += 'Data: ' + self._visual_sum + self._joint_sum + self._act_sum + diag_sum

            #html += '<hr size="2"><br>\n' 
            html += self._visual_html + '<hr size="2">\n'
            html += self._joint_html + '<hr size="2">\n'
            html += self._act_html + '<hr size="2">\n'
            html += diag_html + '<hr size="2">\n'
            
            r = TestResultRequest()
            r.html_result = html
            r.text_summary = summary
            r.plots = []
            
            if self._is_ok and self._visual_ok and self._joints_ok and self._acts_ok and not self._timeout:
                r.result = r.RESULT_PASS
            else:
                r.result = r.RESULT_HUMAN_REQUIRED
                
            rospy.wait_for_service('test_result', 5)

            try:
                self.result_srv.call(r)
                self.has_sent_result = True
            except Exception, e:
                rospy.logerr('Caught exception sending OK service. %s' % traceback.format_exc())
        except Exception, e:
            self.send_failure_call('checkout_robot', traceback.format_exc())

    def act_data(self, act_datas):
        # Don't need position data, gives actuator position in encoder ticks
        try:
            self._acts_ok = True

            html = '<table border=\"1\" cellpadding=\"2\" cellspacing=\"0\">\n'
            html += '<tr><td><b>Index</b></td><td><b>Name</b></td><td><b>Enabled</b></td><td><b>ID</b></td></tr>\n'
            for act_data in act_datas:
                index = act_data.index
                name = act_data.name
                id = act_data.id
                if act_data.enabled:
                    enabled = '<div class=\"pass\">OK</div>'
                else:
                    enabled = '<div class=\"warn\">FAIL</div>'
                    self._acts_ok = False
                html += '<tr><td>%d</td><td>%s</td><td>%s</td><td>%d</td></tr>\n' % (index, name, enabled, id)
            
            html += '</table>\n'
            
            if self._acts_ok:
                self._act_sum = 'Acutators: OK. ' 
            else:
                self._act_sum = 'Actuators: FAIL! '

            self._act_html = '<p><b>Actuator Data</b></p><p>%s</p><br>\n' % self._act_sum
            self._act_html += html

        except Exception, e:
            self.send_failure_call('actuator_data', traceback.format_exc())
            

    def joint_data(self, jnt_datas):
        # Type doesn't work
        try:
            self._joints_ok = True
            self._calibrated = True   
            
            html = '<table border=\"1\" cellpadding=\"2\" cellspacing=\"0\">\n'
            html += '<tr><td><b>Index</b></td><td><b>Name</b></td><td><b>Type</b></td><td><b>Is Cal?</b></td><td><b>Has Safety?</b></td></tr>\n'
            for jnt_data in jnt_datas:
                id = jnt_data.index
                name = jnt_data.name
                type = jnt_data.type
                #print 'Cal %s' % jnt_data.is_cal
                if jnt_data.is_cal == 1:
                    cal = '<div class=\"pass\">OK</div>'
                elif name.endswith('wheel_joint'):
                    cal = '<div class=\"pass\">Wheel</div>'
                elif name == 'base_joint':
                    cal = '<div class=\"pass\">Base</div>'
                else:
                    cal = '<div class=\"warn\">NO</div>'
                    self._joints_ok = False
                    self._calibrated = False
                    
                if jnt_data.has_safety:
                    safe = 'OK'
                elif type == 'Continuous': 
                    # Cont. joints don't have safety min/max
                    safe = '<div class=\"pass\">Continuous</div>'
                elif type == 'Fixed':
                    safe = '<div class=\"pass\">Fixed</div>'
                elif type =='Planar' and name=='base_joint':
                    safe = 'Base'
                else:
                    safe = '<div class=\"warn\">NO</div>'
                    self._joints_ok = False
                    
                html += '<tr><td>%d</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>\n' % (id, name, type, cal, safe)

            html += '</table>\n'
                        
            if self._joints_ok:
                self._joint_sum = 'Joint states: OK. '
            else:
                if self._calibrated:
                    self._joint_sum = 'Joint states: FAIL. '
                else:
                    self._joint_sum = 'Joints states: FAIL, not all calibrated. '
            self._joint_html = '<p><b>Joint Data</b></p><p>%s</p><br>\n' % self._joint_sum
            self._joint_html += html
        except Exception, e:
            self.send_failure_call('joint_data', traceback.format_exc())

             
if __name__ == '__main__':
    try:
        checkout = RobotCheckout()
        rospy.spin()
    except Exception, e:
        print 'Caught exception in robot checkout'
        traceback.print_exc()

    print 'Quitting robot checkout'
