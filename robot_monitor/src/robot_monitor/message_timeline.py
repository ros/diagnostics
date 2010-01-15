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

##\author Josh Faust

PKG = "robot_monitor"

from collections import deque
import rospy
import roslib

import wx
import os
import os.path

def clamp(val, min, max):
    if (val < min):
        return min
    if (val > max):
        return max
    
    return val

class ColoredTimeline(wx.Control):
    def __init__(self, parent, min_val, max_val, val, color_callback):
        wx.Control.__init__(self, parent, wx.ID_ANY)
        
        self._min = min_val
        self._max = max_val
        self._val = val
        self._color_callback = color_callback
        
        self.SetMinSize(wx.Size(-1, 16))
        self.SetMaxSize(wx.Size(-1, 16))

        self._timeline_marker_bitmap = wx.Bitmap(os.path.join(roslib.packages.get_pkg_dir(PKG), 'icons/timeline_marker.png'), wx.BITMAP_TYPE_PNG)
        
        self.Bind(wx.EVT_PAINT, self.on_paint)
        self.Bind(wx.EVT_SIZE, self.on_size)
        self.Bind(wx.EVT_MOUSE_EVENTS, self.on_mouse)
        
        self._left_down = False

        self.SetToolTip(wx.ToolTip("Drag to rewind messages"))
        
    def SetValue(self, val):
        self._val = clamp(int(val), self._min, self._max)
        self.Refresh()
        
    def GetValue(self):
        return self._val
    
    def SetRange(self, min_val, max_val):
        self._min = min_val
        self._max = max_val
        self._val = clamp(self._val, min, max)
        self.Refresh()
        
    def on_size(self, event):
        self.Refresh()
        
    def on_paint(self, event):
        dc = wx.PaintDC(self)
        dc.Clear()
        
        is_enabled = self.IsEnabled()

        (width, height) = self.GetSizeTuple()
        length = self._max + 1 - self._min
        value_size = width / float(length)
        for i in xrange(0, length):
            if (is_enabled):
                color = self._color_callback(i + self._min)
            else:
                color = wx.LIGHT_GREY
            end_color = wx.Colour(0.6 * color.Red(), 0.6 * color.Green(), 0.6 * color.Blue())
            dc.SetPen(wx.Pen(color))
            dc.SetBrush(wx.Brush(color))
            start = i * value_size
            end = (i + 1) * value_size
            dc.GradientFillLinear(wx.Rect(start, 0, end, height), color, end_color, wx.SOUTH)
            
            if (i > 0):
                dc.SetPen(wx.BLACK_PEN)
                dc.DrawLine(start, 0, start, height)
            
        marker_x = (self._val - 1) * value_size + (value_size / 2.0) - (self._timeline_marker_bitmap.GetWidth() / 2.0)
        dc.DrawBitmap(self._timeline_marker_bitmap, marker_x, 0, True)
        
    def _set_val_from_x(self, x):
        (width, height) = self.GetSizeTuple()
        # determine value from mouse click
        length = self._max + 1 - self._min
        value_size = width / float(length)
        self.SetValue(x / value_size + 1)
        
    def on_mouse(self, event):
        if (event.LeftDown()):
            self._left_down = True
        elif (event.LeftUp()):
            self._left_down = False
            self._set_val_from_x(event.GetX())
            wx.PostEvent(self.GetEventHandler(), wx.PyCommandEvent(wx.EVT_COMMAND_SCROLL_CHANGED.typeId, self.GetId()))
        
        if (self._left_down):
            self._set_val_from_x(event.GetX())
            wx.PostEvent(self.GetEventHandler(), wx.PyCommandEvent(wx.EVT_COMMAND_SCROLL_THUMBTRACK.typeId, self.GetId()))

class MessageTimeline(wx.Panel):
    def __init__(self, parent, count, topic, type, message_callback, color_callback, pause_callback):
        wx.Panel.__init__(self, parent, wx.ID_ANY)
        
        self._count = count
        self._message_callback = message_callback
        self._color_callback = color_callback
        self._pause_callback = pause_callback
        self._queue = deque()
        
        sizer = wx.BoxSizer(wx.HORIZONTAL)
        self._timeline = ColoredTimeline(self, 1, 1, 1, self._get_color_for_value)
        sizer.Add(self._timeline, 1, wx.ALIGN_CENTER_VERTICAL|wx.ALL, 5)
        self._pause_button = wx.ToggleButton(self, wx.ID_ANY, "Pause")
        self._pause_button.SetToolTip(wx.ToolTip("Pause message updates"))
        sizer.Add(self._pause_button, 0, wx.ALIGN_CENTER_VERTICAL|wx.ALL, 5)
        self.SetSizer(sizer)
        
        self._pause_button.Bind(wx.EVT_TOGGLEBUTTON, self.on_pause)
        self._paused = False
        self._last_msg = None
        
        self._tracking_latest = True
        self.Layout()
        
        self._last_val = 2
        
        self._timeline.Bind(wx.EVT_COMMAND_SCROLL_THUMBTRACK, self.on_slider_scroll)
        self._timeline.Bind(wx.EVT_COMMAND_SCROLL_CHANGED, self.on_slider_scroll)
        
        self._subscriber = None
        if (topic is not None):
            self._subscriber = rospy.Subscriber(topic, type, self.callback)
            
        self._message_receipt_callback = None
        
    def set_message_receipt_callback(self, cb):
        self._message_receipt_callback = cb
        
    def __del__(self):
        if (self._subscriber is not None):
            self._subscriber.unregister()
            
    def enable(self):
        wx.Panel.Enable(self)
        self._timeline.Enable()
        self._pause_button.Enable()
        
    def disable(self):
        wx.Panel.Disable(self)
        self._timeline.Disable()
        self._pause_button.Disable()
        self.unpause()
        
    def clear(self):
        self._queue.clear()
        self._timeline.SetRange(1, 1)
        self._timeline.SetValue(1)
        
    def is_paused(self):
        return self._paused
    
    def pause(self):
        self._paused = True
        self._pause_button.SetBackgroundColour(wx.Colour(123, 193, 255))
        self._pause_button.SetToolTip(wx.ToolTip("Resume message updates"))
        
        if (self._pause_callback is not None):
            self._pause_callback(True)
            
        self._pause_button.SetValue(True)
    
    def unpause(self):
        if (self._pause_callback is not None):
            self._pause_callback(False)
            
        self._pause_button.SetBackgroundColour(wx.NullColour)
        self._pause_button.SetToolTip(wx.ToolTip("Pause message updates"))
        self._paused = False
        if (self._last_msg is not None):
            self._tracking_latest = True
            self._new_msg(self._last_msg)
            
        self._pause_button.SetValue(False)
        
    def on_pause(self, event):
        if (event.IsChecked()):
            self.pause()
        else:
            self.unpause()
                
    def on_slider_scroll(self, event):
        val = self._timeline.GetValue() - 1
        if (val == self._last_val):
            return
        
        if (val >= len(self._queue)):
            return
        
        self._last_val = val
        
        if (not self._paused and self._pause_callback is not None):
            self._pause_callback(True)
            
        self._pause_button.SetValue(True)
        self._pause_button.SetBackgroundColour(wx.Colour(123, 193, 255))
        self._paused = True
        self._tracking_latest = False
        
        msg = self._queue[val]
        
        self._message_callback(msg)
        
    def callback(self, msg):
        wx.CallAfter(self._new_msg, msg)
        
    def add_msg(self, msg):
        self._new_msg(msg)
        
    def _new_msg(self, msg):
        self._last_msg = msg
        if (self._message_receipt_callback is not None):
            self._message_receipt_callback(msg)
        if (self._paused):
            return
        
        self._queue.append(msg)
        if (len(self._queue) > self._count):
            self._queue.popleft()
            
        new_len = len(self._queue)
            
        self._timeline.SetRange(1, new_len)
        
        if (self._tracking_latest):
            self._timeline.SetValue(new_len)
            self._message_callback(msg)
            
    def _get_color_for_value(self, val):
        if (val == 1 and len(self._queue) == 0):
            return wx.LIGHT_GREY
        
        return self._color_callback(self._queue[val - 1])
            
    def get_messages(self):
        return self._queue
    
