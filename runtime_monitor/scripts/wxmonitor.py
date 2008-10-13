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

## A basic node to listen to and display incoming diagnostic messages using wx

import rostools
rostools.update_path('runtime_monitor')

import sys
import rospy
from robot_msgs.msg import *

import wx
import threading, time

import runtime_monitor
from runtime_monitor.monitor_panel import *

NAME = 'runtime_monitor'

class MainWindow(wx.Frame):
    def __init__(self, parent, id, title):
        wx.Frame.__init__(self, parent, wx.ID_ANY, title)
        self.filemenu = wx.Menu()
        self.filemenu.Append(wx.ID_EXIT, "E&xit"," Exit the program")
        self.menubar = wx.MenuBar()
        self.menubar.Append(self.filemenu,"&File")
        self.SetMenuBar(self.menubar)
        wx.EVT_MENU(self, wx.ID_EXIT, self.on_exit)
        
        panel = MonitorPanel(self)
        panel.set_new_errors_callback(self.on_error)
        
        self.SetSize(wx.Size(750,450))


    def on_exit(self, e):
        self.Close(True)
            
    def on_error(self):
        self.Raise()
    
def listener():
    app = wx.PySimpleApp()
    rospy.ready(NAME, anonymous=True)
    
    frame = MainWindow(None, -1, "Runtime Monitor")
    frame.Show()
    
    app.MainLoop()
        
        
if __name__ == '__main__':
    try:
        listener()
    except KeyboardInterrupt, e:
        pass
    print "exiting"
