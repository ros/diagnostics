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
roslib.load_manifest('life_test')

import sys, os, math, string
from datetime import datetime
import csv
import traceback
from time import sleep, mktime, strftime, localtime
import threading
from socket import gethostname

import wx
import wx.aui
from wx import xrc

import rospy, roslaunch
from std_srvs.srv import *
from robot_msgs.msg import *
import runtime_monitor
from runtime_monitor.monitor_panel import MonitorPanel

import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.base import MIMEBase
from email import Encoders


class LifeTest:
    def __init__(self, short_serial, test_name, desc, cycle_rate, test_type, launch_file):
        self._short_serial = short_serial
        self._name = test_name
        self._desc = desc
        self._cycle_rate = cycle_rate
        self._launch_script = launch_file
        self._test_type = test_type

class TestRecord:
    def __init__(self, test, serial):
        self._start_time = rospy.get_time()
        self._cum_seconds = 0
        self._last_update_time = rospy.get_time()
        self._was_running = False
        self._was_launched = False
        self._num_alerts = 0
        self._num_halts = 0

        self._rate = test._cycle_rate
        self._serial = serial
        self._test_name = test._name

        self._log = {}

        csv_name = strftime("%m%d%Y_%H%M%S", localtime(self._start_time)) + '_' + str(self._serial) + '_' + self._test_name + '.csv'
        csv_name = csv_name.replace(' ', '_').replace('/', '__')
        self.log_file = os.path.join(roslib.packages.get_pkg_dir('life_test'), 'logs/%s' % csv_name)

        
        log_csv = csv.writer(open(self.log_file, 'wb'))
        log_csv.writerow(['Time', 'Status', 
                               'Elapsed (s)', 'Active (s)', 
                               'Cycles', 'Cycle Rate', 'Notes'])

    def get_elapsed(self):
        elapsed = rospy.get_time() - self._start_time
        return elapsed

    def get_cum_time(self):
        return self._cum_seconds

    def get_duration_str(self, duration):
        hrs = max(math.floor(duration / 3600), 0)
        min = max(math.floor(duration / 6), 0) / 10 - hrs * 60

        return "%dhr, %.1fm" % (hrs, min)

    def get_active_str(self):
        return self.get_duration_str(self._cum_seconds)

    def get_elapsed_str(self):
         return self.get_duration_str(self.get_elapsed())

    def get_cycles(self):
        cycles = self._rate * self._cum_seconds
        return cycles
            
    def update(self, launched, running, stale, note):
        if running and not launched:
            rospy.logerr('Reported impossible state of running and not launched')
            return 0, ''

        if stale and running:
            rospy.logerr('Reported impossible state of running and stale')
            return 0, ''
        
        if self._was_running and running:
            self._cum_seconds = rospy.get_time() - self._last_update_time + self._cum_seconds

        alert = 0 # 0 - None, 1 - Notify, 2 - alert
        msg = ''
        state = 'Running'

        if launched and (not running) and stale:
            state = 'Halted, stale'
        elif launched and (not running):
            state = 'Halted'
        elif not launched:
            state = 'Stopped'

        if (not self._was_launched) and launched:
            alert = 1
            msg = "Test launched."
        elif self._was_launched and (not launched):
            alert = 1
            msg = "Test shut down."

        elif self._was_running and (not running):
            alert = 2
            if stale:
                msg = "Test has gone stale!"
            else:
                msg = "Test has stopped running!"
        elif (not self._was_running) and running:
            alert = 1
            msg = "Test has restarted and is running."

            self._num_halts += 1

        if alert:
            self._num_alerts += 1

        self._was_running = running
        self._was_launched = launched
        self._last_update_time = rospy.get_time()

        if alert or note != '':
            self.write_csv_row(self._last_update_time, state, msg, note)
            self._log[rospy.get_time()] = msg + ' ' + note

        return alert, msg

    def write_csv_row(self, update_time, state, msg, note):
        log_msg = msg + ' ' + note
        log_msg = log_msg.replace(',', ';')

        time_str = strftime("%m/%d/%Y %H:%M:%S", localtime(update_time))

        log_csv = csv.writer(open(self.log_file, 'ab'))
        log_csv.writerow([time_str, state, 
                               self.get_elapsed(), self.get_cum_time(), 
                               self.get_cycles(), self._rate, 
                               log_msg])
      
    def csv_filename(self):
        return self.log_file
        

class TestMonitorPanel(wx.Panel):
    def __init__(self, parent, manager, test, serial):
        wx.Panel.__init__(self, parent)

        self._manager = manager

        self._mutex = threading.Lock()


        self._diag_sub = None
        self._diags = []
        
        # Set up test and loggers
        self._test = test
        self._serial = serial
        self._record = TestRecord(test, serial)

        # Set up panel
        xrc_path = os.path.join(roslib.packages.get_pkg_dir('life_test'), 'xrc/gui.xrc')

        self._panel = xrc.XmlResource(xrc_path).LoadPanel(self, 'test_panel')
        #self._test_data_panel = self._manager._xrc.LoadPanel(self, 'test_data')
        self._test_desc = xrc.XRCCTRL(self._panel, 'test_desc')
        self._test_desc.SetValue(self._test._desc)

        self._launch_button = xrc.XRCCTRL(self._panel, 'launch_test_button')
        self._launch_button.Bind(wx.EVT_BUTTON, self.launch_test)

        self._test_machine_ctrl = xrc.XRCCTRL(self._panel, 'test_machine_ctrl')
        
        self._end_cond_type = xrc.XRCCTRL(self._panel, 'end_cond_type')
        self._end_cond_type.SetStringSelection('Continuous')
        self._end_cond_type.Bind(wx.EVT_CHOICE, self.on_end_choice)

        self._end_cond_type_label = xrc.XRCCTRL(self._panel, 'duration_label')

        self._test_duration_ctrl = xrc.XRCCTRL(self._panel, 'test_duration_ctrl')
        
        self._close_button = xrc.XRCCTRL(self._panel, 'close_button')
        self._close_button.Bind(wx.EVT_BUTTON, self.on_close)
        
        self._status_bar = xrc.XRCCTRL(self._panel, 'test_status_bar')

        self._reset_button = xrc.XRCCTRL(self._panel, 'reset_motors_button')
        self._reset_button.Bind(wx.EVT_BUTTON, self.on_reset_motors)

        self._halt_button = xrc.XRCCTRL(self._panel, 'halt_motors_button')
        self._halt_button.Bind(wx.EVT_BUTTON, self.on_halt_motors)

        self._stop_button = xrc.XRCCTRL(self._panel, 'stop_test_button')
        self._stop_button.Bind(wx.EVT_BUTTON, self.stop_test)

        self._user_log = xrc.XRCCTRL(self._panel, 'user_log_input')
        self._user_submit = xrc.XRCCTRL(self._panel, 'user_submit_button')
        self._user_submit.Bind(wx.EVT_BUTTON, self.on_user_entry)

        self._done_time_ctrl = xrc.XRCCTRL(self._panel, 'done_time_ctrl')
        self._total_cycles_ctrl = xrc.XRCCTRL(self._panel, 'total_cycles_ctrl')
        
        self._elapsed_time_ctrl = xrc.XRCCTRL(self._panel, 'elapsed_time_ctrl')
        self._active_time_ctrl = xrc.XRCCTRL(self._panel, 'active_time_ctrl')

        self._log_ctrl = xrc.XRCCTRL(self._panel, 'test_log')

        self._test_log_window = xrc.XRCCTRL(self._panel, 'test_log_window')
        self._send_log_button = xrc.XRCCTRL(self._panel, 'send_test_log_button')
        self._send_log_button.Bind(wx.EVT_BUTTON, self.on_send_test_log)

        
        # Add runtime to the pane...
        self._notebook = xrc.XRCCTRL(self._panel, 'test_data_notebook')
        wx.CallAfter(self.create_monitor)

        self._sizer = wx.BoxSizer(wx.HORIZONTAL)
        self._sizer.Add(self._panel, 1, wx.EXPAND)
        self.SetSizer(self._sizer)
        self.Layout()

        self._test_team = [ "watts@willowgarage.com", "stanford@willowgarage.com" ]
        
        self._machine = None
        self._current_log = {}
        self._diag_msgs = {}

        self._is_running = False

        # Launches test, call stop to kill it
        self._test_launcher = None

        # Test log data
        self._test_complete = False

        # Timeout for etherCAT diagnostics
        self.timer = wx.Timer(self, 1)
        self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)
        self.last_message_time = rospy.get_time()
        self.timeout_interval = 2.0
        self._is_stale = True

        # Timer for invent logging
        self.invent_timer = wx.Timer(self, 2)
        self.Bind(wx.EVT_TIMER, self.on_invent_timer, self.invent_timer)
        self._last_invent_time = rospy.get_time()
        self.invent_timeout = 600
        self._is_invent_stale = True
        self._invent_note_id = None
 
        self.update_controls()
        self.on_end_choice()
    
    def create_monitor(self):
        self._monitor_panel = MonitorPanel(self._notebook)
        self._monitor_panel.SetSize(wx.Size(400, 500))
        self._notebook.AddPage(self._monitor_panel, "Runtime Monitor")

        
    def __del__(self):
        # Somehow calling log function in destructor
        print 'Stopping launches'
        self.stop_test()
        
    def is_launched(self):
        return self._test_launcher is not None

    def on_end_choice(self, event = None):
        #self._mutex.acquire()
        choice = self._end_cond_type.GetStringSelection()

        # Set test_duration_ctrl units also
        label = choice.lower()
        if choice == 'Continuous':
            label = 'N/A'
        self._end_cond_type_label.SetLabel(choice.lower())

        # Set spin ctrl based on end type
        self._test_duration_ctrl.SetValue(0)
        active_time = self._record.get_cum_time()

        cycles = self._record.get_cycles()

        if choice == 'Hours':
            hrs = math.ceil((active_time / 3600))
            self._test_duration_ctrl.SetRange(hrs, 168) # Week
            self._test_duration_ctrl.SetValue(hrs)
        elif choice == 'Minutes':
            min = math.ceil((active_time / 60))
            self._test_duration_ctrl.SetRange(min, 600) # 10 Hrs
            self._test_duration_ctrl.SetValue(min + 10)
        elif choice == 'Cycles':
            cycle_val = cycles + math.ceil(self._test._cycle_rate * 300) # 5 min
            self._test_duration_ctrl.SetRange(cycles, 100000) # Long time
            self._test_duration_ctrl.SetValue(cycle_val)
        else:
            self._test_duration_ctrl.SetRange(0, 0) # Can't change limits
            self._test_duration_ctrl.SetValue(0)

        #self._mutex.release()

    def on_user_entry(self, event):
        entry = self._user_log.GetValue()
        msg = 'OPERATOR: ' + entry
        self.log(msg)
        self._user_log.Clear()
        self._user_log.SetFocus()

    def on_send_test_log(self, event):
        names = wx.GetTextFromUser('Enter recipient names, separated by commas: NAME1,NAME2 (without "@willowgarage.com").', 'Enter recipient', '', self)

        names = names.split(',')
        for name in names:
            if name.find('@') < 0:
                name = name + '@willowgarage.com'

        self.notify_operator(3, 'Test log requested by operator.', string.join(names, ','))

    def on_close(self, event):
        #self.stop_test()
        self.log('Closing down test.')
        self.update_invent()
        self.record_test_log()
        self.notify_operator(1, 'Test closing.')
        self._manager.close_tab(self._serial)


    def update_test_record(self, note = ''):
        alert, msg = self._record.update(self.is_launched(), self._is_running, self._is_stale, note)
        if alert > 0:
            #self.log(msg)
            self._current_log[rospy.get_time()] = msg
            self._manager.log_test_entry(self._test._name, self._machine, msg)
            
            wx.CallAfter(self.display_logs)
            self.notify_operator(alert, msg)


    def calc_run_time(self):
        end_condition = self._end_cond_type.GetStringSelection()
        
        duration = self._test_duration_ctrl.GetValue()

        if end_condition == 'Hours':
            return duration * 3600
        if end_condition == 'Minutes':
            return duration * 60
        if end_condition == 'Seconds':
            return duration
        if end_condition == 'Cycles':
            return duration / self._test._cycle_rate

        else: #if end_condition == 'Continuous':
            return 10**9 # Roughly 30 years

    def calc_remaining(self):
        total_sec = self.calc_run_time()
        cum_sec = self._record.get_cum_time()
    
        return total_sec - cum_sec
        

    def on_invent_timer(self, event):
        if rospy.get_time() - self._last_invent_time > self.invent_timeout and self.is_launched():
            self.update_invent()
        
        self.invent_timer.Start(self.invent_timeout * 500, True)


    def update_invent(self):
        # rospy.logerr('Updating invent')

        # Reset timer, last time
        self.invent_timer.Start(self.invent_timeout * 500, True)
        self._last_invent_time = rospy.get_time()

        # Don't log anything if we haven't launched
        if not self.is_launched() and not self._invent_note_id:
            return

        hrs_str = self._record.get_elapsed_str()
        cycles = self._record.get_cycles()

        stats = "Stats: Total cycles %.1f, elapsed time %s." % (cycles, hrs_str)
        
        if self.is_launched() and self._is_running:
            note = "Test running %s at %s Hz. " % (self._test._name, self._test._cycle_rate)
        elif self.is_launched() and not self._is_running:
            note = "Test %s is halted. " % self._test._name
        else:
            note = "Test %s finished. CSV name: %s. " % (self._test._name, os.path.basename(self._record.csv_filename()))

        # rospy.logerr('Updated note successfully')
        self._invent_note_id = self._manager._invent_client.setNote(self._serial, note + stats, self._invent_note_id)
        #print 'Logging in invent for reference %s: %s' % (self._serial, note + stats)

    def record_test_log(self):
        try:
            # Adds log csv to invent
            if self._record.get_cum_time() == 0:
                return # Don't log test that hasn't run
            
            # rospy.logerr('Writing CSV file')
            
            f = open(self._record.csv_filename(), 'rb')
            csv = f.read()
            self._manager._invent_client.add_attachment(self._serial, os.path.basename(self._record.csv_filename()), 'text/csv', csv)
            f.close()
            # print 'Wrote CSV of test log to invent'
            
            summary_name = strftime("%m%d%Y_%H%M%S", localtime(self._record._start_time)) + '_summary.html'
            self._manager._invent_client.add_attachment(self._serial, summary_name, 'text/html', self.make_html_test_summary())
        except Exception, e:
            rospy.logerr('Unable to submit to invent. %s' % traceback.format_exc())
                                                
        
    def start_timer(self):
        self.timer.Start(1000, True)
        
    def on_timer(self, event):
        self._mutex.acquire()
        
        interval = rospy.get_time() - self.last_message_time
        
        if interval > self.timeout_interval or interval < 0:
            # Make EtherCAT status stale
            self._is_running = False
            self._is_stale = True

            self.update_test_record()
            self.stop_if_done()
            self.update_controls(3)
        else:
            self.start_timer()
            self._is_stale = False
            
        self._mutex.release()
        self.Refresh()

    def update_controls(self, level = 3, msg = 'None'):
        # Assumes it has the lock
        if not self.is_launched():
            self._status_bar.SetValue("Launch to display status")
            self._status_bar.SetBackgroundColour("White")
        elif level == 0:
            self._status_bar.SetValue("Test Running: OK")
            self._status_bar.SetBackgroundColour("Light Green")
        elif level == 1:
            self._status_bar.SetValue("Test Warning! Warning: %s" % msg)
            self._status_bar.SetBackgroundColour("Orange")
        elif level == 2:
            self._status_bar.SetValue("Error in EtherCAT Master: %s" % msg)
            self._status_bar.SetBackgroundColour("Red")
        else:
            self._status_bar.SetBackgroundColour("White")
            self._status_bar.SetValue("EtherCAT Master Stale!")
        
        self._reset_button.Enable(self.is_launched())
        self._halt_button.Enable(self.is_launched())
        self._stop_button.Enable(self.is_launched())

        # FIX
        remaining = self.calc_remaining()
        remain_str = "N/A" 
        if remaining < 10**6:
            remain_str = self._record.get_duration_str(remaining)
        self._done_time_ctrl.SetValue(remain_str)

        cycles = "%.1f" % self._record.get_cycles()
        self._total_cycles_ctrl.SetValue(cycles)

        self._test_log_window.Freeze()
        (x, y) = self._test_log_window.GetViewStart()
        self._test_log_window.SetPage(self.make_html_cycle_log_table())
        self._test_log_window.Scroll(x, y)
        self._test_log_window.Thaw()

        self._active_time_ctrl.SetValue(self._record.get_active_str())
        self._elapsed_time_ctrl.SetValue(self._record.get_elapsed_str())
        
        self._test_machine_ctrl.Enable(not self.is_launched())
        self._launch_button.Enable(not self.is_launched())
        self._close_button.Enable(not self.is_launched())
        
    def print_cur_log(self):
        kys = dict.keys(self._current_log)
        kys.sort()

        log_str = ''
        for ky in kys:
            log_str += strftime("%m/%d/%Y %H:%M:%S: ", 
                                localtime(ky)) + self._current_log[ky] + '\n'

        return log_str

    def stop_if_done(self):
        remain = self.calc_remaining()

        # Make sure we've had five consecutive seconds of 
        # negative time before we shutdown
        # Can this be part of test record?
        if remain < 0:
            self.notify_operator(1, 'Test complete!')
            self._test_complete = True
            self.stop_test()
        
    def stop_test(self, event = None):
        if self.is_launched():
            self.on_halt_motors(None)
            self._test_launcher.stop()
            self._manager.test_stop(self._machine)
            self.log('Stopping test launch')

        self._diag_sub = None

        self._test_launcher = None
        self._is_running = False
        
        if event is not None: # In GUI thread
            self.update_controls()


    def log(self, msg):
        self.update_test_record(msg)
        self._current_log[rospy.get_time()] = msg
        self._manager.log_test_entry(self._test._name, self._machine, msg)

        wx.CallAfter(self.display_logs)

    def display_logs(self):
        # Make them bottom of log visible...
        try:
            self._mutex.acquire()
        except:
            return
      
        self._log_ctrl.AppendText(self.print_cur_log())
        self._log_ctrl.Refresh()
        self._log_ctrl.Update()
        self._current_log = {}
        self._mutex.release()

    def diag_callback(self, message):
        #print 'Acquiring mutex for diagnostic callback'
        self._mutex.acquire()
        #print 'Diagnostic mutex acquired'
        self._diags.append(message)
        self._mutex.release()
        wx.CallAfter(self.new_msg)
    
    def new_msg(self):
        self._mutex.acquire()

        # Simple and hopefully effective way to see if test is still running
        etherCAT_master_name = 'EtherCAT Master'
        level_dict = { 0: 'OK', 1: 'Warn', 2: 'Error' }
        try:
            for msg in self._diags:
                for stat in msg.status:
                    if stat.name == etherCAT_master_name:
                        self.last_message_time = rospy.get_time()
                        self.start_timer()

                        self._is_running = (stat.level == 0)
                        self._is_stale = False
                        self.update_test_record()
                        self.stop_if_done()
                        self.update_controls(stat.level, stat.message)

            self._diags = []

        except Exception, e:
            rospy.logerr('Caught exception processing diagnostic msg.\nEx: %s' % traceback.format_exc())
          
        self._mutex.release()
        self.Refresh()
    
    
    def make_launch_script(self):
        launch = '<launch>\n'
        launch += '<group ns="%s">' % self._machine
        launch += '<remap from="/diagnostics" to="/%s/diagnostics" />' % self._machine
        # Init machine
        launch += '<machine name="test_host_root" user="root" address="%s" ' % self._machine
        launch += 'ros-root="$(env ROS_ROOT)" ros-package-path="$(env ROS_PACKAGE_PATH)" default="never"/>'
        launch += '<machine name="test_host" address="%s" ' % self._machine
        launch += 'ros-root="$(env ROS_ROOT)" ros-package-path="$(env ROS_PACKAGE_PATH)"  />'

        launch += '<include file="$(find life_test)/%s" />' % self._test._launch_script
        launch += '</group>\n</launch>'

        return launch


    # Put in master file
    # Add subscriber to diagnostics
    # Launch file, subscribe diagnostics
    def launch_test(self, event):
        # Prompt ARE YOU SURE
        dialog = wx.MessageDialog(self, 'Are you sure you want to launch?', 'Confirm Launch', wx.OK|wx.CANCEL)
        if dialog.ShowModal() != wx.ID_OK:
            return


        # Get machine, end condition, etc
        machine = self._test_machine_ctrl.GetStringSelection()

        if not self._manager.test_start_check(machine):
            wx.MessageBox('Machine in use, select again!', 'Machine in use', wx.OK|wx.ICON_ERROR, self)
            return
        
        
        self._machine = machine

        self.log('Launching test %s on machine %s.' % (self._test._name, self._machine))

        config = roslaunch.ROSLaunchConfig()
        try:
            loader = roslaunch.XmlLoader()
            loader.load_string(self.make_launch_script(), config)
            self._test_launcher = roslaunch.ROSLaunchRunner(config)
            self._test_launcher.launch()
        except roslaunch.RLException, e:
            self.log('Failed to launch script')
            self.log(traceback.format_exc())
            self._manager.test_stop(self._machine)
            self._machine = None
            return 


        local_diagnostics = '/' + self._machine + '/diagnostics'
        self._is_running = True
        self.update_invent()
        self._monitor_panel.change_diagnostic_topic(local_diagnostics)

        self._mutex.acquire()
        self.update_controls()
        self._mutex.release()

        self._diag_sub = rospy.Subscriber(local_diagnostics, DiagnosticMessage, self.diag_callback)


    def on_halt_motors(self, event = None):
        try:
            self.log('Halting motors.')
            # Call halt motors service on NAME_SPACE/pr2_etherCAT
            halt_srv = rospy.ServiceProxy(self._machine + '/halt_motors', Empty)
            halt_srv()

        except Exception, e:
            rospy.logerr('Exception on halt motors. %s' % traceback.format_exc())

    def on_reset_motors(self, event = None):
         try:
             self.log('Reseting motors')
             reset = rospy.ServiceProxy(self._machine + '/reset_motors', Empty)
             reset()

         except:
            rospy.logerr('Exception on reset motors. %s' % traceback.format_exc())
      
    # 
    # Loggers and data processing -> Move to record class or elsewhere
    # 
    def get_test_team(self):
        # Don't email everyone it's debugging on NSF
        if os.environ['USER'] == 'watts' and gethostname() == 'nsf':
            return 'watts@willowgarage.com'

        return string.join(self._test_team, ",")

    def line_summary(self, msg):
        machine_str = self._machine
        if not self._machine:
            machine_str = 'NONE'
        return "%s Test %s on machine %s of %s" % (msg, self._test._name, machine_str, self._serial)

    def notify_operator(self, level, alert_msg, recipient = None):
        # Don't notify if we haven't done anything
        if self._record.get_cum_time() == 0 and not self.is_launched() and level == 1:
            return

        sender = 'test.notify@willowgarage.com'
        header = '--Test Notify--'
        if level == 2:
            sender = 'test.alerts@willowgarage.com'
            header = '--Test Alert--'
        elif level == 3:
            sender = 'test.reports@willowgarage.com'
            header = '--Test Report--'
        
        try:
            if not recipient:
                recipient = self.get_test_team()

            msg = MIMEMultipart('alternative')
            msg['Subject'] = header + self.line_summary(alert_msg)
            msg['From'] = sender
            msg['To'] = recipient

            msg.attach(MIMEText(self.make_html_test_summary(alert_msg), 'html'))
            
            log_csv = open(self._record.csv_filename(), 'rb')
            log_data = log_csv.read()
            log_csv.close()

            part = MIMEBase('application', 'octet-stream')
            part.set_payload(log_data)
            Encoders.encode_base64(part)
            part.add_header('Content-Disposition', 'attachment; filename="%s"' 
                            % os.path.basename(self._record.csv_filename()))
            
            msg.attach(part)

            s = smtplib.SMTP('localhost')
            s.sendmail(sender, recipient, msg.as_string())
            s.quit()

            return True
        except Exception, e:
            rospy.logerr('Unable to send mail! %s' % traceback.format_exc())
            self.log('Unable to send mail! %s' % traceback.format_exc())
            return False


    def make_html_cycle_log_table(self):
        log_csv = csv.reader(open(self._record.csv_filename(), 'rb'))
        
        is_first = True

        html = '<html>\n<table border="1" cellpadding="2" cellspacing="0">\n'
        for row in log_csv:
            html += '<tr>'
            for val in row:  
                if unicode(val).isnumeric():
                    val = "%.2f" % float(val)
                
                if is_first:
                    html += '<td><b>%s</b></td>' % val
                else:
                    html += '<td>%s</td>' % val

            is_first = False
            html += '</tr>\n'

        html += '</table>\n</html>'

        return html

    # Dump these into test_result class of some sort
    def make_html_test_summary(self, alert_msg = ''):
        html = '<html><head><title>Test Log: %s of %s</title>' % (self._test._name, self._serial)
        html += '<style type=\"text/css\">\
body { color: black; background: white; }\
div.error { background: red; padding: 0.5em; border: none; }\
div.warn { background: orange: padding: 0.5em; border: none; }\
div.pass { background: green; padding: 0.5em; border: none; }\
strong { font-weight: bold; color: red; }\
em { font-style:normal; font-weight: bold; }\
</style>\
</head>\n<body>\n'

        html += '<H2 align=center>Test Log: %s of %s</H2>\n' % (self._test._name, self._serial)
        
        if alert_msg != '':
            html += '<H3>Alert: %s</H3><br>\n' % alert_msg

        if self._test_complete:
            html += '<H3>Test Completed Successfully</H3>\n'
        else:
            if self.is_launched() and not self._is_running:
                html += '<H3>Test Status: Launched, Halted</H3>\n'
            elif self.is_launched() and self._is_running:
                html += '<H3>Test Status: Launched, Running</H3>\n'
            else:
                html += '<H3>Test Status: Shutdown</H3>\n'

        html += '<H4>Test Info</H4>\n'
        html += '<p>Description: %s</p>\n<br>' % self._test._desc
        html += self.make_test_info_table()

        # Make results table
        html += '<hr size="3">\n'
        html += '<H4>Test Results</H4>\n'
        
        html += self.make_record_table()
        

        # Make log table
        html += '<hr size="3">\n'
        html += '<H4>Test Log</H4>\n'
        html += self.make_log_table()
        html += '<hr size="3">\n'
        html += '</body></html>'

        return html

    def make_test_info_table(self):
        html = '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += self.make_table_row('Test Name', self._test._name)
        html += self.make_table_row('Serial', self._serial)
        html += self.make_table_row('Cycle Rate', self._test._cycle_rate)
        html += self.make_table_row('Test Type', self._test._test_type)
        html += '</table>\n'

        return html


    def make_table_row(self, label, value):
        return '<tr><td><b>%s</b></td><td>%s</td></tr>\n' % (label, value)

    def make_record_table(self):
        if not self._record:
            return '<p>No test record, test may have been aborted.</p>\n'
            
        html = '<table border="1" cellpadding="2" cellspacing="0">\n'
        time_str = strftime("%m/%d/%Y %H:%M:%S", localtime(self._record._start_time))
        html += self.make_table_row('Start Time', time_str)
        html += self.make_table_row('Elapsed Time', self._record.get_elapsed_str())
        cycle_str = "%.1f" % self._record.get_cycles()
        html += self.make_table_row('Total Cycles', cycle_str)
        html += self.make_table_row('Active Time', self._record.get_active_str())
        
        html += '</table>\n'
        return html

    def make_log_table(self):
        if self._record._log is None or len(dict.keys(self._record._log)) == 0:
            return '<p>No test log!</p>\n'

        html = '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Time</b></td><td><b>Entry</b></td></tr>\n'
        
        kys = dict.keys(self._record._log)
        kys.sort()
        for ky in kys:
            time_str = strftime("%m/%d/%Y %H:%M:%S", localtime(ky))
            html += self.make_table_row(time_str, self._record._log[ky])
            
        html += '</table>\n'

        return html
