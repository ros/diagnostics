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

import roslib
roslib.load_manifest(PKG)

import sys
import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import wx
from wx import xrc
from wx import html

import threading, time
import cStringIO
import copy

class TreeItem(object):
  def __init__(self, status, tree_id):
    self.status = status
    self.mark = False
    self.stale = False
    self.tree_id = tree_id
        
class MonitorPanel(wx.Panel):
  def __init__(self, parent):
    wx.Panel.__init__(self, parent, wx.ID_ANY)
    
    self._mutex = threading.Lock()
    
    xrc_path = roslib.packages.get_pkg_dir(PKG) + '/xrc/runtime_monitor_generated.xrc'
    
    self._xrc = xrc.XmlResource(xrc_path)
    self._real_panel = self._xrc.LoadPanel(self, 'RuntimeMonitorInnerPanel')
    sizer = wx.BoxSizer(wx.VERTICAL)
    sizer.Add(self._real_panel, 1, wx.EXPAND)
    self.SetSizer(sizer)

    self._tree_control = xrc.XRCCTRL(self._real_panel, "_tree_control")
    self._html_control = xrc.XRCCTRL(self._real_panel, "_html_control")
    
    image_list = wx.ImageList(16, 16)
    self._error_image_id = image_list.AddIcon(wx.ArtProvider.GetIcon(wx.ART_ERROR, wx.ART_OTHER, wx.Size(16, 16)))
    self._warning_image_id = image_list.AddIcon(wx.ArtProvider.GetIcon(wx.ART_WARNING, wx.ART_OTHER, wx.Size(16, 16)))
    self._ok_image_id = image_list.AddIcon(wx.ArtProvider.GetIcon(wx.ART_TICK_MARK, wx.ART_OTHER, wx.Size(16, 16)))
    self._tree_control.AssignImageList(image_list)
    self._root_id = self._tree_control.AddRoot("Root")
    self._stale_id = self._tree_control.AppendItem(self._root_id, "Stale (0)", self._error_image_id)
    self._errors_id = self._tree_control.AppendItem(self._root_id, "Errors (0)", self._error_image_id)
    self._warnings_id = self._tree_control.AppendItem(self._root_id, "Warnings (0)", self._warning_image_id)
    self._ok_id = self._tree_control.AppendItem(self._root_id, "Ok (0)", self._ok_image_id)
    
    self._tree_control.Bind(wx.EVT_TREE_SEL_CHANGED, self.on_item_selected)
    self._tree_control.Bind(wx.EVT_TREE_KEY_DOWN, self.on_item_key_down)
    
    self._name_to_item = {}
    
    self._new_errors_callback = None
    
    self._timer = wx.Timer(self)
    self.Bind(wx.EVT_TIMER, self.on_timer)
    self._timer.Start(5000)
    
    self._subscriber = rospy.Subscriber("/diagnostics", DiagnosticArray, self.diagnostics_callback)
    
    self._messages = []
    self._used_items = 0
    
  def __del__(self):
    self._subscriber.unregister()
      
  def change_diagnostic_topic(self, topic):
    if len(topic) == 0:
      self.reset_monitor()
      return

    self._subscriber.unregister()
    self._subscriber = rospy.Subscriber(str(topic), DiagnosticArray, self.diagnostics_callback)
    self.reset_monitor()


  def reset_monitor(self):
    self._name_to_item = {} # Reset all stale topics
    self._messages = []
    self.clear_tree()

  def clear_tree(self):
    self._tree_control.DeleteChildren(self._stale_id)
    self._tree_control.DeleteChildren(self._errors_id)
    self._tree_control.DeleteChildren(self._warnings_id)
    self._tree_control.DeleteChildren(self._ok_id)

  def diagnostics_callback(self, message):
    self._mutex.acquire()
    
    self._messages.append(message)
    
    self._mutex.release()
    
    wx.CallAfter(self.new_message)
      
  def new_message(self):
    # The panel can have messages in the queue after it's destroyed
    # If it's been destroyed, just ignore it
    try:
      self._mutex.acquire()
    except:
      return

    had_errors = False
    
    for message in self._messages:
      for status in message.status:
        was_selected = False
        had_item = False
        if (self._name_to_item.has_key(status.name)):
          item = self._name_to_item[status.name]
          had_item = True
          if (self._tree_control.GetSelection() == item.tree_id):
            was_selected = True
          
          if (item.status.level == 2 and status.level != 2):
            had_errors = True
            
          self.update_item(item, status, was_selected)
        else:
          self.create_item(status, was_selected, True)
          if (status.level == 2):
              had_errors = True
    
    self._messages = []
    
    self._mutex.release()
    
    if (had_errors and self._new_errors_callback != None):
      self._new_errors_callback()
        
    self.Refresh()
      
  def update_item(self, item, status, was_selected):
    change_parent = False
    if (item.status.level != status.level):
      change_parent = True
      
    if (change_parent):
      self._tree_control.Delete(item.tree_id)
      
      if (status.level == 0):
        parent_id = self._ok_id
      elif (status.level == 1):
        parent_id = self._warnings_id
      elif (status.level == -1):
        parent_id = self._stale_id
      else:
        parent_id = self._errors_id
      
      id = self._tree_control.AppendItem(parent_id, status.name + ": " + status.message)
      item.tree_id = id
      self._tree_control.SetPyData(id, item)
      
      if (status.level > 1 or status.level == -1):
        self._tree_control.Expand(parent_id)
      
      self._tree_control.SortChildren(parent_id)
        
      if (was_selected):
        self._tree_control.SelectItem(item.tree_id)
      
    else:
      self._tree_control.SetItemText(item.tree_id, status.name + ": " + status.message)
      
    item.status = status
    
    if (was_selected):
      self.fillout_info(item.tree_id)
      
    item.mark = True
    
  def create_item(self, status, select, expand_if_error):
    if (status.level == 0):
      parent_id = self._ok_id
    elif (status.level == 1):
      parent_id = self._warnings_id
    elif (status.level == -1):
      parent_id = self._stale_id
    else:
      parent_id = self._errors_id
    
    id = self._tree_control.AppendItem(parent_id, status.name + ": " + status.message)
    item = TreeItem(status, id)
    self._tree_control.SetPyData(id, item)
    
    self._name_to_item[status.name] = item
    
    self._tree_control.SortChildren(parent_id)
    
    if (select):
      self._tree_control.SelectItem(id)
        
    if (expand_if_error and (status.level > 1 or status.level == -1)):
      self._tree_control.Expand(parent_id)
        
    self.update_root_labels()
    
    item.mark = True
    
    return item
          
  def fillout_info(self, id):
    item = self._tree_control.GetPyData(id)
    if (item == None):
      return
      
    status = item.status
    
    self._html_control.Freeze()
    s = cStringIO.StringIO()
    
    s.write("<html><body>")
    s.write("<b>Component</b>: %s<br>\n" % (status.name))
    s.write("<b>Message</b>: %s<br>\n" % (status.message))
    s.write("<b>Hardware ID</b>: %s<br><br>\n\n" % (status.hardware_id)) 
    
    s.write('<table border="1" cellpadding="2" cellspacing="0">')
    for value in status.values:
      value.value = value.value.replace("\n", "<br>")
      s.write("<tr><td><b>%s</b></td> <td>%s</td></tr>\n" % (value.key, value.value))
        
    s.write("</table></body></html>")
        
    (x, y) = self._html_control.GetViewStart()
    self._html_control.SetPage(s.getvalue())
    self._html_control.Scroll(x, y)
        
    self._html_control.Thaw()
          
  def on_item_selected(self, event):
    self.fillout_info(event.GetItem())
      
  def on_item_key_down(self, event):
    id = self._tree_control.GetSelection()
    if (id == None):
      event.Skip()
      return
    
    key = event.GetKeyCode()
    
    if (key == wx.WXK_DELETE):
      item = self._tree_control.GetPyData(id)
      if (item != None):
          self._tree_control.Delete(id)
          del self._name_to_item[item.status.name]
    else:
      event.Skip()
      
    self.update_root_labels()
    self.Refresh()
      
      
  def on_timer(self, event):
    for name, item in self._name_to_item.iteritems():
      id = item.tree_id
      if (item != None):
        if (not item.mark):
          was_selected = False
          if (self._tree_control.GetSelection() == id):
            was_selected = True
          
          new_status = copy.deepcopy(item.status)
          new_status.level = -1
          self.update_item(item, new_status, was_selected)
            
        item.mark = False
    
        
    self.update_root_labels()
    self.Refresh()
      
  def set_new_errors_callback(self, callback):
    self._new_errors_callback = callback

  def get_num_errors(self):
    return self._tree_control.GetChildrenCount(self._errors_id, False) + self._tree_control.GetChildrenCount(self._stale_id, False)
  
  def get_num_warnings(self):
    return self._tree_control.GetChildrenCount(self._warnings_id, False)
  
  def get_num_ok(self):
    return self._tree_control.GetChildrenCount(self._ok_id, False)

  def update_root_labels(self):
    self._tree_control.SetItemText(self._ok_id, "Ok (%s)" % (self._tree_control.GetChildrenCount(self._ok_id, False)))
    self._tree_control.SetItemText(self._warnings_id, "Warnings (%s)" % (self._tree_control.GetChildrenCount(self._warnings_id, False)))
    self._tree_control.SetItemText(self._errors_id, "Errors (%s)" % (self._tree_control.GetChildrenCount(self._errors_id, False)))
    self._tree_control.SetItemText(self._stale_id, "Stale (%s)" % (self._tree_control.GetChildrenCount(self._stale_id, False)))
