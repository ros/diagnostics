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

# Author: Kevin Watts

PKG = 'robot_monitor'

import roslib; roslib.load_manifest(PKG)

import sys, os
import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import wx
from wx import xrc
from wx import html
from wx import richtext

import copy

import cStringIO

stat_dict = {0: 'OK', 1: 'Warning', 2: 'Error', 3: 'Stale' }

##\brief View status messages in pop-up window
##
## Allows users to view details of status in popup window
##\todo Add play/pause buttons, message buffer
class StatusViewer(wx.Panel):
    ##\param parent StatusViewerFrame : Parent frame
    ##\param name str : Full topic name to listen to
    ##\param manager RobotMonitor : Manager updates frame, notified on close
    def __init__(self, parent, name, manager):
        wx.Panel.__init__(self, parent, wx.ID_ANY)
        
        self._sizer = wx.BoxSizer(wx.VERTICAL)
        
        self._text_ctrl = richtext.RichTextCtrl(self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, wx.TE_READONLY)
        self._sizer.Add(self._text_ctrl, 1, wx.EXPAND)
        self._pause_button = wx.ToggleButton(self, wx.ID_ANY, "Pause")
        self._sizer.Add(self._pause_button, 0, wx.ALIGN_RIGHT)
        
        self.SetSizer(self._sizer)
        
        self._text_ctrl.SetFocus()
        self._pause_button.Bind(wx.EVT_TOGGLEBUTTON, self.on_pause)

        self._manager = manager
        self._name = name
        
        self._paused = False
        self._last_status = None
        
        self._default_style = self._text_ctrl.GetDefaultStyle()
        self._basic_style = self._text_ctrl.GetBasicStyle()
        
        tabs = [600, 800, 1000]
        self._default_style.SetFlags(wx.TEXT_ATTR_TABS)
        self._default_style.SetTabs(tabs)
        
    ##\brief Destructor removes viewer from manager's update list
    def __del__(self):
        self._manager.remove_viewer(self._name)
        
    def on_pause(self, event):
        self._pause_button.SetBackgroundColour(wx.NullColour)
        if (event.IsChecked()):
            self._paused = True
            self._pause_button.SetBackgroundColour(wx.Colour(0xff, 0x33, 0x22))
        else:
            self._paused = False
            if (self._last_status is not None):
                self._write_status(self._last_status)

    def set_status(self, status):
        self._last_status = status
        if (self._paused):
            return
        
        self._write_status(status)
        
    def _set_kv(self, key, value):
        self._text_ctrl.BeginBold()
        self._text_ctrl.WriteText("%s: "%(key))
        self._text_ctrl.EndBold()
        self._text_ctrl.WriteText(value)
        self._text_ctrl.Newline()

    ##\brief Write status as HTML, like runtime monitor
    def _write_status(self, status):
        self._text_ctrl.Freeze()
        self._text_ctrl.Clear()
        self._text_ctrl.SetBasicStyle(self._basic_style)
        self._text_ctrl.SetDefaultStyle(self._default_style)
        self._set_kv("Full name", status.name)
        self._set_kv("Component", status.name.split('/')[-1])
        self._set_kv("Hardware ID", status.hardware_id)
        self._set_kv("Level", stat_dict[status.level])
        self._set_kv("Message", status.message)
        self._text_ctrl.Newline()
        
        for value in status.values:
            self._set_kv(value.key, value.value)
            
        self._text_ctrl.EndAllStyles()    
        self._text_ctrl.Thaw()

##\brief Frame views status messages in separate window
##
##\todo Don't initialize it on top of main frame somehow
class StatusViewerFrame(wx.Frame):
    ##\param parent RobotMonitorFrame : Parent frame
    ##\param name str : Full topic name
    ##\param manager RobotMonitor : Manager of frame
    ##\param title str: Frame title
    def __init__(self, parent, name, manager, title):
        wx.Frame.__init__(self, parent, wx.ID_ANY, title)
        
        self.panel = StatusViewer(self, name, manager)
        
        self.Bind(wx.EVT_CHAR, self.on_char)
        self.panel._text_ctrl.Bind(wx.EVT_CHAR, self.on_char)
        
    def on_char(self, evt):
        if (evt.GetKeyCode() == wx.WXK_ESCAPE):
          self.Close()
        else:
          evt.Skip()
        
