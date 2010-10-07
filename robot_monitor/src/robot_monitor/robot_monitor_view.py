# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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

##\brief rxbag plugin as robot_monitor

PKG = 'robot_monitor'
import roslib; roslib.load_manifest(PKG)
import rospy

import wxversion
WXVER = '2.8'
if wxversion.checkInstalled(WXVER):
    wxversion.select(WXVER)
else:
    print >> sys.stderr, 'This application requires wxPython version %s' % WXVER
    sys.exit(1)
import wx

from rxbag import TopicMessageView

import robot_monitor_panel

class RobotMonitorView(TopicMessageView):
    name = 'Robot Monitor'
    
    def __init__(self, timeline, parent):
        TopicMessageView.__init__(self, timeline, parent)

        self.monitor_panel = robot_monitor_panel.RobotMonitorPanel(self.parent, rxbag=True)
        self.monitor_panel.SetPosition((1, 0))
        
    def message_viewed(self, bag, msg_details):
        TopicMessageView.message_viewed(self, bag, msg_details)
        
        if msg_details:
            topic, msg, t = msg_details

            wx.CallAfter(self.monitor_panel.new_message, msg)

    def message_cleared(self):
        TopicMessageView.message_cleared(self)
        
        wx.CallAfter(self.monitor_panel.reset_monitor)

    def close(self):
        pass


    def on_size(self, event):
        size = self.parent.GetClientSize()

        self.resize(*size)
        self.monitor_panel.SetSize((size[0], size[1]))

    def on_right_down(self, event):
        self.parent.PopupMenu(RuntimeMonitorPopupMenu(self.parent, self), event.GetPosition())

class RobotMonitorPopupMenu(wx.Menu):
    def __init__(self, parent, robot_view):
        wx.Menu.__init__(self)

        self.robot_view = robot_view

        clear_item = wx.MenuItem(self, wx.NewId(), 'Clear Monitor')
        self.AppendItem(clear_item)
        self.Bind(wx.EVT_MENU, lambda e: self.robot_view.reset_monitor(), id=clear_item.GetId())
