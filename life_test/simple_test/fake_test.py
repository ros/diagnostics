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

import roslib
roslib.load_manifest('life_test')
import wx
import sys

from robot_msgs.msg import DiagnosticMessage, DiagnosticStatus
from std_srvs.srv import *

import os
import time

import rospy

import threading
from wx import xrc

class FakeTestFrame(wx.Frame):
    def __init__(self, parent):
        wx.Frame.__init__(self, parent, wx.ID_ANY, 'Fake Test')
        
        self.diag_pub = rospy.Publisher('diagnostics', DiagnosticMessage)

        self._diag_timer = wx.Timer(self, 1)
        self.Bind(wx.EVT_TIMER, self.on_timer, self._diag_timer)
        self._last_publish = rospy.get_time()
        self._diag_timer.Start(500, True)
        
        # Load XRC
        xrc_path = os.path.join(roslib.packages.get_pkg_dir('life_test'), 'xrc/gui.xrc')

        self._panel = xrc.XmlResource(xrc_path).LoadPanel(self, 'fake_panel')
        self._pub_check_box = xrc.XRCCTRL(self._panel, 'publish_check_box')
        self._level_choice = xrc.XRCCTRL(self._panel, 'level_choice')
        
        self._reset_srv = rospy.Service('reset_motors', Empty, self.on_reset)
        self._halt_srv = rospy.Service('halt_motors', Empty, self.on_halt)
        
    def on_halt(self, srv):
        print 'Halting'
        wx.CallAfter(self.set_level, 2)
        return EmptyResponse()

    def on_reset(self, srv):
        wx.CallAfter(self.set_level, 0)
        return EmptyResponse()

    def set_level(self, val):
        print 'Setting level'
        self._level_choice.SetSelection(val)

    def on_timer(self, event = None):
        print 'Making message'
        self._diag_timer.Start(500, True)

        level_dict = { "OK": 0, "Warn": 1, "Error": 2 }

        level =  self._level_choice.GetSelection()
        # level_dict[self._level_choice.GetStringSelection()]
        print 'Level %s' % level
        choice = self._level_choice.GetStringSelection()
        print 'Choice %s' % choice
     
        if self._pub_check_box.IsChecked():
            self.publish_diag(level, str(choice))


    def publish_diag(self, level, choice):
        msg = DiagnosticMessage()
        stat = DiagnosticStatus()
        msg.status.append(stat)

        stat.level = level
        stat.name = 'EtherCAT Master' # So ghetto
        stat.message = choice #'OK'
     
        self.diag_pub.publish(msg)

class FakeTestApp(wx.App):
    def OnInit(self):
        rospy.init_node("fake_test")
        self._frame = FakeTestFrame(None)
        self._frame.SetSize(wx.Size(200, 100))
        self._frame.Layout()
        self._frame.Centre()
        self._frame.Show(True)

        return True

if __name__ == '__main__':
    try:
        app = FakeTestApp(0)
        app.MainLoop()
    except:
        print 'Caught in FakeTestApp'
        import traceback
        traceback.print_exc()
