#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

##\author Kevin Watts, Josh Faust

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

from message_timeline import MessageTimeline

stat_dict = {0: 'OK', 1: 'Warning', 2: 'Error', 3: 'Stale' }
color_dict = {0: wx.Colour(85, 178, 76), 1: wx.Colour(222, 213, 17), 2: wx.Colour(178, 23, 46), 3: wx.Colour(178, 23, 46)}

class SnapshotFrame(wx.Frame):
    def __init__(self, parent, name):
        wx.Frame.__init__(self, parent, wx.ID_ANY, "Snapshot of %s"%(name))
        
        self._sizer = wx.BoxSizer(wx.VERTICAL)
        self._text_ctrl = richtext.RichTextCtrl(self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, wx.TE_READONLY)
        self._sizer.Add(self._text_ctrl, 1, wx.EXPAND)
        
class Item():
    def __init__(self):
        self.name = None
    
    def __cmp__(self, other):
        if (isinstance(other, str)):
            return cmp(self.name, other)
        else:
            return cmp(self.name, other.name)

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
        
        self._sizer = wx.BoxSizer(wx.VERTICAL)
        
        self._text_ctrl = richtext.RichTextCtrl(self, wx.ID_ANY, wx.EmptyString, wx.DefaultPosition, wx.DefaultSize, wx.TE_READONLY)
        self._sizer.Add(self._text_ctrl, 1, wx.EXPAND)
        
        bottom_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self._timeline = MessageTimeline(self, 30, None, None, self._write_status, self._get_color_for_message, None)
        bottom_sizer.Add(self._timeline, 1, wx.EXPAND|wx.ALL, 5)
        
        self._snapshot_button = wx.Button(self, wx.ID_ANY, "Snapshot")
        self._snapshot_button.Bind(wx.EVT_BUTTON, self._on_snapshot)
        self._snapshot_button.SetToolTip(wx.ToolTip("Freeze data in new window"))
        bottom_sizer.Add(self._snapshot_button, 0, wx.ALL|wx.ALIGN_CENTER_VERTICAL, 5)
        
        self._sizer.Add(bottom_sizer, 0, wx.EXPAND)
        
        self.SetSizer(self._sizer)
        
        self._text_ctrl.SetFocus()

        self._manager = manager
        self._name = name
        
        self._default_style = self._text_ctrl.GetDefaultStyle()
        self._basic_style = self._text_ctrl.GetBasicStyle()
        
        self.Bind(wx.EVT_CHAR, self.on_char)
        self._text_ctrl.Bind(wx.EVT_CHAR, self.on_char)
        
        self.Bind(wx.EVT_CLOSE, self._on_close)
        
        self._last_values = {}
        self._last_status = None
        
        self._items = []
        
    def _on_close(self, event):
        event.Skip()
        self._manager.remove_viewer(self._name)
        
    def _on_snapshot(self, event):
        snapshot = SnapshotFrame(self, self._name)
        snapshot._text_ctrl.SetValue(self._text_ctrl.GetValue())
        snapshot.Show(True)
        snapshot.Raise()
        snapshot.Center()

    def set_status(self, status):
        if (self._timeline.IsEnabled()):
            self._timeline.add_msg(status)
        else:
            self._write_status(status)
        
    def _set_kv(self, key, value, changed=False):
        self._text_ctrl.BeginBold()
        self._text_ctrl.WriteText("%s: "%(key))
        self._text_ctrl.EndBold()
        
        if (not changed):
            self._text_ctrl.WriteText(value)
        else:
            self._text_ctrl.BeginBold()
            self._text_ctrl.BeginTextColour(wx.Colour(0xaa, 0x77, 0x00))
            self._text_ctrl.WriteText(value)
            self._text_ctrl.EndTextColour()
            self._text_ctrl.EndBold()
        self._text_ctrl.Newline()

    def _write_status(self, status):
        self._text_ctrl.Freeze()
        self._text_ctrl.BeginSuppressUndo()
        
        # SetCaretPosition and GetCaretPosition was only added to wxPython in 2.8.10 apparently, even though they've been in wx for ages
        has_caret_accessors = True
        try:
            getattr(self._text_ctrl, 'SetCaretPosition')
        except:
            has_caret_accessors = False
            
        #has_caret_accessors = False

        if (has_caret_accessors):
            self._text_ctrl.SetCaretPosition(0)
        else:
            self._text_ctrl.Clear()
        
        self._text_ctrl.SetBasicStyle(self._basic_style)
        self._text_ctrl.SetDefaultStyle(self._default_style)
        self._set_kv("Full name", status.name)
        self._set_kv("Component", status.name.split('/')[-1])
        self._set_kv("Hardware ID", status.hardware_id)
        self._set_kv("Level", stat_dict[status.level], self._last_status is not None and self._last_status.level != status.level)
        self._set_kv("Message", status.message, self._last_status is not None and self._last_status.message != status.message)
        self._text_ctrl.Newline()
        
        for value in status.values:
            changed = False
            if (self._last_values.has_key(value.key) and self._last_values[value.key] != value.value):
                changed = True
                
            self._set_kv(value.key, value.value, changed)
            self._last_values[value.key] = value.value
            
        self._text_ctrl.EndAllStyles()
        
        if (has_caret_accessors):
            self._text_ctrl.Remove(self._text_ctrl.GetCaretPosition(), self._text_ctrl.GetLastPosition())
            
        self._text_ctrl.EndSuppressUndo()
        self._text_ctrl.Thaw()
        
        self._last_status = status
        
    def _get_color_for_message(self, msg):
        return color_dict[msg.level]
        
    def on_char(self, evt):
        if (evt.GetKeyCode() == wx.WXK_ESCAPE):
          self.Close()
        else:
          evt.Skip()
          
    def disable_timeline(self):
        self._timeline.disable()
        self._timeline.clear()
        
    def enable_timeline(self):
        self._timeline.enable()
        
    def get_name(self):
        return self._name
