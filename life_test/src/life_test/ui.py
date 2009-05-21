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

### Author: Kevin Watts

import roslib
import roslib.packages
roslib.load_manifest('life_test')

import rospy
import roslaunch
import roslaunch.pmon

import os
import sys
import datetime
import glob
import wx
import time
from wx import xrc
from wx import html

import thread
from xml.dom import minidom


from cStringIO import StringIO
import struct

import traceback

from invent_client.invent_client import Invent

from life_test import *

import threading

class TestManagerFrame(wx.Frame):
    def __init__(self, parent):
        wx.Frame.__init__(self, parent, wx.ID_ANY, "Test Manager")
        
        self._mutex = threading.Lock()

        self.create_menu_bar()

        # Load tests by short serial number
        self._tests_by_serial = {}

        # Load XRC
        xrc_path = os.path.join(roslib.packages.get_pkg_dir('life_test'),  'xrc/gui.xrc')
        self._xrc = xrc.XmlResource(xrc_path)

        # Get real username/password for invent here...
        self._invent_client = None # Invent('watts', 'willow')
        
        self._active_machines = []

        self.load_tests_from_file()

        self._main_panel = self._xrc.LoadPanel(self, 'manager_panel')

        self._tab_ctrl = xrc.XRCCTRL(self._main_panel, 'test_tab_control')
        self._start_panel = xrc.XRCCTRL(self._tab_ctrl, 'start_panel')
        self._serial_text = xrc.XRCCTRL(self._start_panel, 'serial_text')
        self._start_button = xrc.XRCCTRL(self._start_panel, 'start_button')
        self._start_button.Bind(wx.EVT_BUTTON, self.on_start)

        self._log_text = xrc.XRCCTRL(self._start_panel, 'log_text')

        self.Bind(wx.EVT_CLOSE, self.on_close)
        self._serial_text.SetFocus()
        
        # Start panel...
        self._test_monitors = {}
        self._active_serials = []

        self._current_log = []

        self.log('Started Test Manager')

    def load_test(self, test, serial):
        # Make notebook page for panel
        # Add panel to notebook
        panel = wx.Panel(self._tab_ctrl, wx.ID_ANY)
        tab_name = test._name + ' ' + serial

        index = len(self._active_serials) + 1

        self._tab_ctrl.AddPage(panel, tab_name, True)

        test_panel = TestMonitorPanel(panel, self, test, serial)
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(test_panel, 1, wx.EXPAND)
        panel.SetSizer(sizer)
        panel.Fit()
        panel.Layout()
        
        self._test_monitors[serial] = test_panel        
        self._active_serials.append(serial)

    def close_tab(self, serial):
        if not serial in self._active_serials:
            rospy.logerr('Requested serial not in active serials: %s' % serial)
            return

        idx = self._active_serials.index(serial)

        if self._tab_ctrl.DeletePage(idx + 1):
            del self._test_monitors[serial]
            del self._active_serials[idx]
        
    def test_start_check(self, machine):
        if machine in self._active_machines:
            return False
        else:
            self._active_machines.append(machine)
            return True

    def test_stop(self, machine):
        if machine in self._active_machines:
            idx = self._active_machines.index(machine)
            del self._active_machines[idx]

    def load_invent_client(self):
        # Loads invent client. Singleton, doesn't construct unless valid login
        if self._invent_client:
            return True

        invent = None
        dialog = self._xrc.LoadDialog(self, 'username_password_dialog')
        xrc.XRCCTRL(dialog, 'login_text').Wrap(300)
        dialog.Layout()
        dialog.Fit()
        username_ctrl = xrc.XRCCTRL(dialog, 'username')
        password_ctrl = xrc.XRCCTRL(dialog, 'password')
        username_ctrl.SetFocus()
        
        # These values don't come through in the xrc file
        username_ctrl.SetMinSize(wx.Size(200, -1))
        password_ctrl.SetMinSize(wx.Size(200, -1))

        while True:
            if (dialog.ShowModal() == wx.ID_OK):
                username = username_ctrl.GetValue()
                password = password_ctrl.GetValue()
            
                invent = Invent(username, password)
                if (invent.login() == False):
                    wx.MessageBox('Please enter a valid username and password.', 'Valid Login Required', wx.OK|wx.ICON_ERROR, self)
                else:
                    self._invent_client = invent
                    return True
            else:
                return False # User doesn't want to log in

        return False
      
 
    def on_start(self, event):
        serial = self._serial_text.GetValue()
        
        if serial in self._active_serials:
            wx.MessageBox('Current component is already testing.', 'Already Testing', wx.OK|wx.ICON_ERROR, self)
            return

        test = self.select_test(serial)

        # If test is none, display message box and return
        if not test:
            wx.MessageBox('No test defined for that serial number or no test selected. Please try again.', 'No test', wx.OK|wx.ICON_ERROR, self)
            return
        
        if not self.load_invent_client():
            wx.MessageBox('You cannot proceed without a valid inventory login.', 'Valid Login Required', wx.OK|wx.ICON_ERROR, self)
            return 

        self._serial_text.Clear()
        self.log('Starting test %s' % test._name)

        wx.CallAfter(self.load_test, test, serial)

    def load_tests_from_file(self):
        test_xml_path = os.path.join(roslib.packages.get_pkg_dir('life_test'), 'tests.xml')
        self._tests = {}
        try:
            doc = minidom.parse(test_xml_path)
        except IOError:
            rospy.logerr('Could not load tests from %s' % test_xml_path)
            sys.exit()

        tests = doc.getElementsByTagName('test')
        for test in tests:
            serial = test.attributes['serial'].value
            name = test.attributes['name'].value
            desc = test.attributes['desc'].value
            cycle_rate = float(test.attributes['rate'].value)
            script = test.attributes['script'].value
            type = test.attributes['type'].value

            if self._tests.has_key(serial):
                self._tests[serial].append(LifeTest(serial, name, desc, cycle_rate, type, script))
            else:
                self._tests[serial] = [ LifeTest(serial, name, desc, cycle_rate, type, script) ]

    def select_string_from_list(self, msg, lst):
        # Load robot selection dialog
        dialog = self._xrc.LoadDialog(self, 'select_test_dialog')
        select_text = xrc.XRCCTRL(dialog, 'select_text')
        select_text.SetLabel(msg)
        test_box = xrc.XRCCTRL(dialog, 'test_list_box')
        test_box.InsertItems(lst, 0)
        
        select_text.Wrap(250)
        
        dialog.Layout()
        dialog.Fit()
        
        # return string of test folder/file to run
        if (dialog.ShowModal() == wx.ID_OK):
            desc = test_box.GetStringSelection()
            dialog.Destroy()
            return desc
        else: 
            dialog.Destroy()
            return None

    def select_test(self, serial):
        short_serial = serial[0:7]
        
        if not self._tests.has_key(short_serial):
            return None
        if len(self._tests[short_serial]) == 1:
            return self._tests[short_serial][0]
        
        # Load select_test_dialog
        tests_by_name = {}
        for test in self._tests[short_serial]:
            tests_by_name[test._name] = test
      
        msg = 'Select test to run.'

        descrips = dict.keys(tests_by_name)
        descrips.sort()
        
        choice = self.select_string_from_list(msg, descrips)
        if choice is None:
            return None
        return tests_by_name[choice]

    def log(self, msg):
        time_str = strftime("%m/%d/%Y %H:%M:%S: ", localtime(rospy.get_time()))

        self._current_log.append(time_str + msg)
        wx.CallAfter(self.update_log_display)

    def log_test_entry(self, test_name, machine, message):
        if not machine:
            machine = 'NONE'

        time_str = strftime("%m/%d/%Y %H:%M:%S: ", localtime(rospy.get_time()))

        log_msg = time_str + 'Machine %s, Test %s. Message: %s' % (machine, test_name, message)

        self._current_log.append(log_msg)
        wx.CallAfter(self.update_log_display)

    def update_log_display(self):
        self._mutex.acquire()
        for log in self._current_log:
            self._log_text.AppendText(log + '\n')
        self._current_log = []
        self._mutex.release()

    def on_close(self, event):
        # Would try/catch here work?
        # Make it safe for bad shutdown

        # Maybe make it so all tests have to be stopped to close...

        # Could just delete monitors
        for value in dict.values(self._test_monitors):
            value.on_close(None)

        self.Destroy()
        
    def on_menu(self, event):
        if (event.GetEventObject() == self._file_menu):
            if (event.GetId() == wx.ID_EXIT):
                self.Close()
                return

    def create_menu_bar(self):
        menubar = wx.MenuBar()
        
        # file menu
        self._file_menu = wx.Menu()
        self._file_menu.Append(wx.ID_EXIT, "E&xit")
        menubar.Append(self._file_menu, "&File")
                
        self.SetMenuBar(menubar)
        
        self.Bind(wx.EVT_MENU, self.on_menu)


class TestManagerApp(wx.App):
    def OnInit(self):
        # Launch roscore
        config = roslaunch.ROSLaunchConfig()
        config.master.auto = config.master.AUTO_RESTART
        
        self._core_launcher = roslaunch.ROSLaunchRunner(config)
        self._core_launcher.launch()

        rospy.init_node("life_test_manager")
        self._frame = TestManagerFrame(None)
        self._frame.SetSize(wx.Size(650, 850))
        self._frame.Layout()
        self._frame.Centre()
        self._frame.Show(True)

        return True

    def OnExit(self):
        self._core_launcher.stop()

if __name__ == '__main__':
    try:
        app = TestManagerApp(0)
        app.MainLoop()
    except Exception, e:
        print 'Caught exception in TestManagerMainLoop'
        traceback.print_exc()
