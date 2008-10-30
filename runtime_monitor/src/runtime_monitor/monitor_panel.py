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

PKG = 'runtime_monitor'

import rostools
rostools.update_path(PKG)

import sys
import rospy
from robot_msgs.msg import *

import wx
from wx import xrc
from wx import html

import threading, time
import StringIO

class MonitorPanel(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent, wx.ID_ANY)
        
        self._mutex = threading.Lock()
        
        xrc_path = rostools.packspec.get_pkg_dir(PKG) + '/xrc/runtime_monitor_generated.xrc'
        
        self._xrc = xrc.XmlResource(xrc_path)
        self._real_panel = self._xrc.LoadPanel(self, 'RuntimeMonitorInnerPanel')
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self._real_panel, 1, wx.EXPAND)
        self.SetSizer(sizer)

        self._tree_control = xrc.XRCCTRL(self._real_panel, "_tree_control")
        self._html_control = xrc.XRCCTRL(self._real_panel, "_html_control")
        
        image_list = wx.ImageList(16,16)
        self._error_image_id = image_list.AddIcon(wx.ArtProvider.GetIcon(wx.ART_ERROR, wx.ART_OTHER, wx.Size(16,16)))
        self._warning_image_id = image_list.AddIcon(wx.ArtProvider.GetIcon(wx.ART_WARNING, wx.ART_OTHER, wx.Size(16,16)))
        self._ok_image_id = image_list.AddIcon(wx.ArtProvider.GetIcon(wx.ART_TICK_MARK, wx.ART_OTHER, wx.Size(16,16)))
        self._tree_control.AssignImageList(image_list)
        self._root_id = self._tree_control.AddRoot("Root")
        self._errors_id = self._tree_control.AppendItem(self._root_id, "Errors (0)", self._error_image_id)
        self._warnings_id = self._tree_control.AppendItem(self._root_id, "Warnings (0)", self._warning_image_id)
        self._ok_id = self._tree_control.AppendItem(self._root_id, "Ok (0)", self._ok_image_id)
        
        self._tree_control.Bind(wx.EVT_TREE_SEL_CHANGED, self.on_item_selected)
        
        self._name_to_id = {}
        
        self._new_errors_callback = None
        
        self._timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer)
        self._timer.Start(10000)
        
        rospy.Subscriber("/diagnostics", DiagnosticMessage, self.diagnostics_callback)
        
        self._messages = []
        self._used_items = 0
        
    def diagnostics_callback(self, message):
        self._mutex.acquire()
        
        self._messages.append(message)
        
        self._mutex.release()
        
        wx.CallAfter(self.new_message)
        
    def new_message(self):
        self._mutex.acquire()
        
        had_errors = False
        
        for message in self._messages:
            for status in message.status:
                was_selected = False
                had_item = False
                if (self._name_to_id.has_key(status.name)):
                    id = self._name_to_id[status.name]
                    had_item = True
                    if (self._tree_control.GetSelection() == id):
                        was_selected = True
                    
                    self._tree_control.Delete(id)
                
                self.create_item(status, was_selected, had_item == False)
                if (status.level == 2 and (not had_item)):
                    had_errors = True
        
        self._messages = []
        
        self._mutex.release()
        
        if (had_errors and self._new_errors_callback != None):
            self._new_errors_callback()
            
        self.Refresh()
        
    def create_item(self, status, select, expand_if_error):
        if (status.level == 0):
            parent_id = self._ok_id
        elif (status.level == 1):
            parent_id = self._warnings_id
        else:
            parent_id = self._errors_id
        
        id = self._tree_control.AppendItem(parent_id, status.name + ": " + status.message)
        self._tree_control.SetPyData(id, status)
        
        self._name_to_id[status.name] = id
        
        self._tree_control.SortChildren(parent_id)
        
        if (select):
            self._tree_control.SelectItem(id)
            
        if (expand_if_error and parent_id == self._errors_id):
            self._tree_control.Expand(parent_id )
            
        self.update_root_labels()
        
        status.mark = True
            
    def fillout_info(self, id):
        status = self._tree_control.GetPyData(id)
        if (status == None):
            return
        
        self._html_control.Freeze()
        s = StringIO.StringIO()
        
        s.write("<html><body>")
        s.write("<b>Component</b>: %s<br>\n"%(status.name))
        s.write("<b>Message</b>: %s<br><br>\n\n"%(status.message))
        
        s.write('<table border="1">')
        for value in status.strings:
            s.write("<tr><td><b>%s</b></td> <td>%s</td></tr>\n" %(value.label, value.value))
        for value in status.values:
            s.write("<tr><td><b>%s</b></td> <td>%s</td></tr>\n" %(value.label, value.value))
            
        s.write("</table></body></html>")
            
        (x, y) = self._html_control.GetViewStart()
        self._html_control.SetPage(s.getvalue())
        self._html_control.Scroll(x, y)
            
        self._html_control.Thaw()
            
    def on_item_selected(self, event):
        self.fillout_info(event.GetItem())
        
    def on_timer(self, event):
        to_delete = []
        for name,id in self._name_to_id.iteritems():
            status = self._tree_control.GetPyData(id)
            if (status != None):
                if (not status.mark):
                    self._tree_control.Delete(id)
                    to_delete.append(name)
                    
                status.mark = False
        
        for name in to_delete:
            del self._name_to_id[name]
            
        self.update_root_labels()
        self.Refresh()
        
    def set_new_errors_callback(self, callback):
        self._new_errors_callback = callback

    def update_root_labels(self):
        self._tree_control.SetItemText(self._ok_id, "Ok (%s)"%( self._tree_control.GetChildrenCount(self._ok_id, False)))
        self._tree_control.SetItemText(self._warnings_id, "Warnings (%s)"%( self._tree_control.GetChildrenCount(self._warnings_id, False)))
        self._tree_control.SetItemText(self._errors_id, "Errors (%s)"%( self._tree_control.GetChildrenCount(self._errors_id, False)))
