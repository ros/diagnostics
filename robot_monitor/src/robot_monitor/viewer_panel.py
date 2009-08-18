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


        xrc_path = os.path.join(roslib.packages.get_pkg_dir(PKG), 'xrc/gui.xrc')
        self._res = xrc.XmlResource(xrc_path)
        self._panel = self._res.LoadPanel(self, 'status_viewer')
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self._panel, 1, wx.EXPAND)
        self.SetSizer(sizer)
 
        self._html_ctrl = xrc.XRCCTRL(self._panel, 'html_ctrl')

        self._manager = manager
        self._name = name
        
    ##\brief Destructor removes viewer from manager's update list
    def __del__(self):
        self._manager.remove_viewer(self._name)

    ##\brief Write status as HTML, like runtime monitor
    def write_status(self, status):
        self._html_ctrl.Freeze()
        s = cStringIO.StringIO()
        
        s.write("<html><body>")
        s.write("<b>Full name</b>: %s<br>\n" % (status.name))
        s.write("<b>Component</b>: %s<br>\n" % (status.name.split('/')[-1]))
        s.write("<b>Hardware ID</b>: %s<br><br>\n\n" % (status.hardware_id))

        s.write("<b>Level</b>: %s<br>\n" % (stat_dict[status.level]))
        s.write("<b>Message</b>: %s<br><br>\n\n" % (status.message))

        s.write('<table border="1" cellpadding="2" cellspacing="0">')
        for value in status.values:
            value.value = value.value.replace("\n", "<br>")
            s.write("<tr><td><b>%s</b></td> <td>%s</td></tr>\n" % (value.key, value.value))
      
        s.write("</table></body></html>")
        
        (x, y) = self._html_ctrl.GetViewStart()
        self._html_ctrl.SetPage(s.getvalue())
        self._html_ctrl.Scroll(x, y)
        
        self._html_ctrl.Thaw()

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
        
    def on_exit(self, e):
        self.Close(True)
