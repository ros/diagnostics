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
#
# Revision $Id$

PKG = 'runtime_monitor'
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

from rxbag import bag_index, msg_view

import monitor_panel



class RuntimeMonitorView(msg_view.TopicMsgView):
    name = 'Runtime Monitor'
    
    def __init__(self, timeline, parent, title, x, y, width, height, max_repaint=None):
        msg_view.TopicMsgView.__init__(self, timeline, parent, title, x, y, width, height, max_repaint)

        self.monitor_panel = monitor_panel.MonitorPanel(self.parent, rxbag=True)
        self.monitor_panel.SetPosition((1, 0))
        
    def message_viewed(self, bag_file, bag_index, topic, stamp, datatype, msg_index, msg):
        msg_view.TopicMsgView.message_viewed(self, bag_file, bag_index, topic, stamp, datatype, msg_index, msg)
        
        if not msg:
            pass
        else:
            self.monitor_panel.add_rxbag_msg(msg, stamp)

    def message_cleared(self):
        msg_view.TopicMsgView.message_cleared(self)
        
        self.monitor_panel._messages = []

    def on_size(self, event):
        size = self.parent.GetClientSize()

        self.resize(*size)
        self.monitor_panel.SetSize((size[0], size[1]))

    def on_right_down(self, event):
        self.parent.PopupMenu(RuntimeMonitorPopupMenu(self.parent, self), event.GetPosition())

    def clear_monitor(self):
        self.monitor_panel.reset_monitor()

class RuntimeMonitorPopupMenu(wx.Menu):
    def __init__(self, parent, runtime_view):
        wx.Menu.__init__(self)

        self.runtime_view = runtime_view

        clear_item = wx.MenuItem(self, wx.NewId(), 'Clear Monitor')
        self.AppendItem(clear_item)
        self.Bind(wx.EVT_MENU, lambda e: self.runtime_view.clear_monitor(), id=clear_item.GetId())
